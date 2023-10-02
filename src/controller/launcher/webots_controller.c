/*
 * Copyright 1996-2023 Cyberbotics Ltd.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     https://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <ctype.h>
#include <dirent.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#ifdef WIN32
#include <io.h>
#include <process.h>
#define F_OK 0
#define access _access
const char PATH_SEPARATOR[] = "\\";
const char ENV_SEPARATOR[] = ";";
#else
const char PATH_SEPARATOR[] = "/";
const char ENV_SEPARATOR[] = ":";
#endif

#define MAX_LINE_BUFFER_SIZE 2048

// Global variables
static char *WEBOTS_HOME;
static char *controller;
static char *controller_path;
static char *controller_extension;
static char *matlab_path;
static char *matlab_args;
static char *current_path;
static int nb_controller_arguments;
static int next_argument_index;

// Environment variables (putenv() requires the string that is set into the environment to exist)
static char *WEBOTS_CONTROLLER_URL;
static char *new_path;
static char *new_windows_path;
static char *new_python_path;
static char *webots_project;
static char *webots_controller_name;
static char *webots_controller_args;
static char *webots_version;

// Removes comments and trailing whitespace from a string.
static void remove_comment(char *string) {
  const char *comment = strchr(string, ';');
  if (comment) {
    const size_t comment_size = strlen(comment);
    const size_t full_line_size = strlen(string);
    const size_t content_size = full_line_size - comment_size;
    string[content_size] = '\0';
    return;
  }
}

// Replaces all occurrences of a character in a string with a different character.
static void replace_char(char *string, char occurrence, char replace) {
  char *current_pos = strchr(string, occurrence);
  while (current_pos) {
    *current_pos = replace;
    current_pos = strchr(current_pos + 1, occurrence);
  }
}

// Removes all occurrences of a character from a string.
static void remove_char(char *string, char occurrence) {
  char *removed = string;
  do {
    while (*removed == occurrence)
      ++removed;
  } while ((*string++ = *removed++));
}

// Replaces all occurrences of a substring in a string with a different string.
static void replace_substring(char **string, const char *substring, const char *replace) {
  char *tmp = *string;
  const size_t substring_size = strlen(substring);
  const size_t replace_size = strlen(replace);

  // Count number of occurences
  int substring_count = 0;
  while (true) {
    char *next = strstr(tmp, substring);
    if (next == NULL)
      break;
    tmp = next + substring_size;
    substring_count++;
  }

  // Adapt memory size, only realloc if more memory is needed
  const size_t new_size = strlen(*string) + substring_count * (replace_size - substring_size) + 1;
  if (new_size > strlen(*string))
    *string = realloc(*string, new_size);

  char *buffer = malloc(new_size);
  char *insert_point = &buffer[0];
  tmp = *string;
  // Replace 'substring' by 'replace'
  while (true) {
    char *next = strstr(tmp, substring);
    if (next == NULL) {
      strcpy(insert_point, tmp);
      break;
    }
    memcpy(insert_point, tmp, next - tmp);
    insert_point += next - tmp;
    memcpy(insert_point, replace, replace_size);
    insert_point += replace_size;
    tmp = next + substring_size;
  }

  // If memory size needed has decreased, realloc memory
  if (new_size < strlen(*string))
    *string = realloc(*string, new_size);

  strcpy(*string, buffer);
  free(buffer);
}

// Inserts a string into another string at a specified index.
static void insert_string(char **string, char *insert, int index) {
  const size_t new_size = strlen(*string) + strlen(insert) + 1;
  char *tmp = strdup(*string);
  *string = realloc(*string, new_size);
  strncpy(*string + index, insert, strlen(insert));
  strncpy(*string + index + strlen(insert), tmp + index, strlen(tmp) - index);
  (*string)[new_size - 1] = '\0';
  free(tmp);
}

// Gets and stores the current working directory path.
static void get_current_path() {
  if (!current_path) {
    current_path = malloc(512);
    getcwd(current_path, 512);
    strcat(current_path, PATH_SEPARATOR);
  }
}

// Gets and stores the path to the Webots installation folder from the WEBOTS_HOME environment variable.
static bool get_webots_home() {
  if (!getenv("WEBOTS_HOME")) {
    fprintf(stderr, "Set the path to your webots installation folder in WEBOTS_HOME environment variable.\n");
    return false;
  } else
    WEBOTS_HOME = malloc(strlen(getenv("WEBOTS_HOME")) + 1);
  strcpy(WEBOTS_HOME, getenv("WEBOTS_HOME"));
  return true;
}

// Gets and stores the path to the latest installed version of Matlab on the system.
static bool get_matlab_path() {
  struct dirent *directory_entry;  // Pointer for directory entry

#ifdef __APPLE__
  const char *matlab_directory = "/Applications/";
  const char *matlab_version_wc = "MATLAB_R20";
  const char *matlab_exec_suffix = "/bin/matlab";
#elif _WIN32
  const char *matlab_directory = "C:\\Program Files\\MATLAB\\";
  const char *matlab_version_wc = "R20";
  const char *matlab_exec_suffix = "\\bin\\matlab.exe";
#elif __linux__
  const char *matlab_directory = "/usr/local/MATLAB/";
  const char *matlab_version_wc = "R20";
  const char *matlab_exec_suffix = "/bin/matlab";
#else
#error "OS not supported!"
#endif

  DIR *directory = opendir(matlab_directory);
  if (directory == NULL) {
#ifdef __APPLE__
    fprintf(stderr, "Could not open Applications folder to search for MATLAB installation. Please specify it manually using "
                    "the option '--matlab-path'.\n");
#else
    fprintf(stderr, "No installation of MATLAB found. Please specify it manually using the option '--matlab-path'.\n");
#endif
    return false;
  }

  // Get latest available Matlab version
  char *latest_version = NULL;
  while ((directory_entry = readdir(directory)) != NULL) {
    const size_t directory_name_size = strlen(directory_entry->d_name) + 1;
    if (strncmp(matlab_version_wc, directory_entry->d_name, strlen(matlab_version_wc)) == 0) {
      if (!latest_version) {
        latest_version = malloc(directory_name_size);
        strncpy(latest_version, directory_entry->d_name, directory_name_size);
      } else if (strcmp(latest_version, directory_entry->d_name) < 0) {
        char *tmp = realloc(latest_version, directory_name_size);
        if (tmp)
          latest_version = tmp;
        strncpy(latest_version, directory_entry->d_name, directory_name_size);
      }
    }
  }
  closedir(directory);
  if (!latest_version) {
    fprintf(stderr, "No installation of MATLAB found. Please specify it manually using the option '--matlab-path'.\n");
    return false;
  }

  const size_t matlab_path_size = snprintf(NULL, 0, "%s%s%s", matlab_directory, latest_version, matlab_exec_suffix) + 1;
  matlab_path = malloc(matlab_path_size);
  sprintf(matlab_path, "%s%s%s", matlab_directory, latest_version, matlab_exec_suffix);
  printf("Using the latest available MATLAB instance: %s\n", matlab_path);

  free(latest_version);
  return true;
}

// Prints the command line options and their descriptions for the Webots controller launcher.
static void print_options() {
  printf(
    "Usage: webots-controller [options] [controller_file] [controller_args]\n\nOptions:\n\n  --help\n    Display this help "
    "message and exit.\n\n  --protocol=<ipc|tcp>\n    Define the protocol to use to communicate between the controller and "
    "Webots.\n    'ipc' is used by default. 'ipc' should be used when Webots is\n    running on the same machine as the extern "
    "controller.\n    'tcp' should be used when connecting to a remote instance of Webots.\n\n  --ip-address=<ip-address>\n    "
    "The IP address of the remote machine on which the Webots instance is running.\n    This option should only be used with "
    "the `tcp` protocol\n    (i.e. remote controllers).\n\n  --port=<port>\n    Define the port to which the controller should "
    "connect.\n    1234 is used by default, as it is the default port for Webots.\n    This setting allows you to connect to a "
    "specific instance of Webots if\n    there are multiple instances running on the target machine.\n    The port of a Webots "
    "instance can be set at its launch.\n\n  --robot-name=<robot-name>\n    Target a specific robot by specifying its name in "
    "case multiple robots wait\n    for an extern controller in the Webots instance.\n\n  --interactive\n    Launch MATLAB "
    "in interactive debugging mode.\n    See https://cyberbotics.com/doc/guide/matlab#using-the-matlab-desktop for\n    more "
    "information.\n\n  --matlab-path=<matlab-path>\n    For MATLAB controllers, this option allows to specify the path to the "
    "\n    executable of a specific MATLAB version.\n    By default, the launcher checks in the default MATLAB installation "
    "folder.\n    See https://cyberbotics.com/doc/guide/running-extern-robot-controllers#running-a-matlab-extern-controller\n"
    "    for more information.\n\n  --stdout-redirect\n    Redirect the stdout of the controller to the Webots console.\n\n  "
    "--stderr-redirect\n    Redirect the stderr of the controller to the Webots console.\n\n");
}

// Parses the command line options for the Webots controller launcher and sets WEBOTS_CONTROLLER_URL envrionment variable.
static bool parse_options(int nb_arguments, char **arguments) {
  if (nb_arguments == 1) {
    fprintf(stderr, "No controller file provided. Please provide an existing controller file as an argument.\n");
    return false;
  }

  controller = NULL;
  matlab_path = NULL;
  matlab_args = NULL;
  char *protocol = NULL;
  char *ip_address = NULL;
  char *port = NULL;
  char *robot_name = NULL;
  bool option_arguments = true;
  for (int i = 1; i < nb_arguments; i++) {
    if (option_arguments && arguments[i][0] == '-') {
      if (strncmp(arguments[i], "--protocol=", 11) == 0) {
        const size_t protocol_size = strlen(arguments[i] + 11) + 1;
        protocol = malloc(protocol_size);
        memcpy(protocol, arguments[i] + 11, protocol_size);
      } else if (strncmp(arguments[i], "--ip-address=", 13) == 0) {
        const size_t ip_address_size = strlen(arguments[i] + 13) + 1;
        ip_address = malloc(ip_address_size);
        memcpy(ip_address, arguments[i] + 13, ip_address_size);
      } else if (strncmp(arguments[i], "--port=", 7) == 0) {
        const size_t port_size = strlen(arguments[i] + 7) + 1;
        port = malloc(port_size);
        memcpy(port, arguments[i] + 7, port_size);
      } else if (strncmp(arguments[i], "--robot-name=", 13) == 0) {
        const size_t robot_name_size = strlen(arguments[i] + 13) + 1;
        robot_name = malloc(robot_name_size);
        memcpy(robot_name, arguments[i] + 13, robot_name_size);
      } else if (strncmp(arguments[i], "--interactive", 13) == 0) {
#ifdef _WIN32
        matlab_args = malloc(strlen("-wait -r") + 1);
        sprintf(matlab_args, "-wait -r");
#else
        matlab_args = malloc(strlen("-r") + 1);
        sprintf(matlab_args, "-r");
#endif
      } else if (strncmp(arguments[i], "--matlab-path=", 14) == 0) {
        const size_t matlab_path_size = strlen(arguments[i] + 14) + 1;
        matlab_path = malloc(matlab_path_size);
        memcpy(matlab_path, arguments[i] + 14, matlab_path_size);
      } else if (strncmp(arguments[i], "--stdout-redirect", 17) == 0) {
        putenv("WEBOTS_STDOUT_REDIRECT=1");
      } else if (strncmp(arguments[i], "--stderr-redirect", 17) == 0) {
        putenv("WEBOTS_STDERR_REDIRECT=1");
      } else if (strncmp(arguments[i], "--help", 6) == 0) {
        print_options();
        return false;
      } else {
        printf("Invalid option '%s'. Try '--help' for more information.\n", arguments[i]);
        return false;
      }
    } else {
      const size_t argument_size = strlen(arguments[i]) + 1;
      if (option_arguments) {
        controller = malloc(argument_size);
        memcpy(controller, arguments[i], argument_size);
        option_arguments = false;
        next_argument_index = i + 1;
        nb_controller_arguments = nb_arguments - i - 1;
      }
    }
  }

  // Check that a controller path has been provided
  if (!controller) {
    fprintf(stderr, "No controller file provided. Please provide an existing controller file as argument.\n");
    return false;
  }

  // If no protocol is given, ipc is used by default
  if (!protocol)
    protocol = strdup("ipc");

  // If no port is given, 1234 is used by default
  if (!port)
    port = strdup("1234");

  // Write WEBOTS_CONTROLLER_URL in function of given options
  const char *robot_separator = robot_name ? "/" : "";
  char *robot_name_string = robot_name ? robot_name : "";
  if (strncmp(protocol, "tcp", 3) == 0) {
    if (!ip_address) {
      fprintf(stderr, "Specify the IP address of the Webots machine to connect to with '--ip-address=' option.\n");
      return false;
    }
    const size_t WEBOTS_CONTROLLER_URL_size = snprintf(NULL, 0, "WEBOTS_CONTROLLER_URL=%s://%s:%s%s%s", protocol, ip_address,
                                                       port, robot_separator, robot_name_string) +
                                              1;
    WEBOTS_CONTROLLER_URL = malloc(WEBOTS_CONTROLLER_URL_size);
    sprintf(WEBOTS_CONTROLLER_URL, "WEBOTS_CONTROLLER_URL=%s://%s:%s%s%s", protocol, ip_address, port, robot_separator,
            robot_name_string);
  } else if (strncmp(protocol, "ipc", 3) == 0) {
    if (ip_address)
      fprintf(stderr, "Skipping IP address for ipc protocol.\n");

    const size_t WEBOTS_CONTROLLER_URL_size =
      snprintf(NULL, 0, "WEBOTS_CONTROLLER_URL=%s://%s%s%s", protocol, port, robot_separator, robot_name_string) + 1;
    WEBOTS_CONTROLLER_URL = malloc(WEBOTS_CONTROLLER_URL_size);
    sprintf(WEBOTS_CONTROLLER_URL, "WEBOTS_CONTROLLER_URL=%s://%s%s%s", protocol, port, robot_separator, robot_name_string);
  } else {
    fprintf(stderr, "Only ipc and tcp protocols are supported.\n");
    return false;
  }
  putenv(WEBOTS_CONTROLLER_URL);

  // Show resulting target options to user
  const char *location = strncmp(protocol, "tcp", 3) == 0 ? "remote" : "local";
  printf("\nThe started controller targets a %s instance (%s protocol) of Webots with port number %s.", location, protocol,
         port);
  strncmp(protocol, "tcp", 3) == 0 ? printf(" The IP address of the remote Webots instance is '%s'.\n", ip_address) :
                                     printf("\n");
  robot_name ? printf("Targeting robot '%s'.\n", robot_name) :
               printf("Targeting the only robot waiting for an extern controller.\n");

  free(protocol);
  free(ip_address);
  free(port);
  free(robot_name);
  return true;
}

/*
This function includes the controller library directory in the relevant OS-specific environment variable:
- For Windows it adds "WEBOTS_HOME\lib\controller;" to the Path environment variable.
- For Linux, it adds "WEBOTS_HOME/lib/controller:" to LD_LIBRARY_PATH.
- For macOS it adds "WEBOTS_HOME/Contents/lib/controller:" to DYLD_LIBRARY_PATH.
*/
static void add_lib_controller_to_path() {
#ifdef _WIN32
  const char *lib_controller = "\\lib\\controller;";
  const char *path_env_variable = "Path";
#elif defined __linux__
  const char *lib_controller = "/lib/controller:";
  const char *path_env_variable = "LD_LIBRARY_PATH";
#elif defined __APPLE__
  const char *lib_controller = "/Contents/lib/controller:";
  const char *path_env_variable = "DYLD_LIBRARY_PATH";
#endif
  const size_t new_path_size =
    snprintf(NULL, 0, "%s=%s%s%s", path_env_variable, WEBOTS_HOME, lib_controller, getenv(path_env_variable)) + 1;
  new_path = malloc(new_path_size);
  sprintf(new_path, "%s=%s%s%s", path_env_variable, WEBOTS_HOME, lib_controller, getenv(path_env_variable));
  putenv(new_path);
}

/*
This function sets the environment variable for java and executable controllers execution.

On Windows, the function adds a new path to the "Path" environment variable by concatenating the WEBOTS_HOME constant, the
'msys64\mingw64\bin' and 'msys64\mingw64\bin\cpp' directories to the current "Path" value.

The function also adds the controller library directory to the Path on all platforms.
*/
static void exec_java_config_environment() {
#ifdef _WIN32
  const size_t new_windows_path_size = snprintf(NULL, 0, "Path=%s\\msys64\\mingw64\\bin;%s\\msys64\\mingw64\\bin\\cpp;%s",
                                                WEBOTS_HOME, WEBOTS_HOME, getenv("Path")) +
                                       1;
  new_windows_path = malloc(new_windows_path_size);
  sprintf(new_windows_path, "Path=%s\\msys64\\mingw64\\bin;%s\\msys64\\mingw64\\bin\\cpp;%s", WEBOTS_HOME, WEBOTS_HOME,
          getenv("Path"));
  putenv(new_windows_path);
#endif
  add_lib_controller_to_path();
}

/*
This function sets the environment variable for python controllers execution.

For Windows it adds "\lib\controller\python;" to PYTHONPATH and adds "\msys64\mingw64\bin\cpp;" to the Path environment
variable. For Linux, it adds "/lib/controller/python:" to PYTHONPATH. For macOS it adds "/Contents/lib/controller/python:" to
PYTHONPATH and adds "/Contents/lib/controller:" to DYLD_LIBRARY_PATH. It sets PYTHONIOENCODING to "UTF-8" on all platforms.
*/
static void python_config_environment() {
#ifdef _WIN32
  const char *python_lib_controller = "\\lib\\controller\\python;";
#elif defined __linux__
  const char *python_lib_controller = "/lib/controller/python:";
#elif defined __APPLE__
  const char *python_lib_controller = "/Contents/lib/controller/python:";
#endif
  const size_t new_python_path_size =
    snprintf(NULL, 0, "PYTHONPATH=%s%s%s", WEBOTS_HOME, python_lib_controller, getenv("PYTHONPATH")) + 1;
  new_python_path = malloc(new_python_path_size);
  sprintf(new_python_path, "PYTHONPATH=%s%s%s", WEBOTS_HOME, python_lib_controller, getenv("PYTHONPATH"));
  putenv(new_python_path);

  char *python_ioencoding = "PYTHONIOENCODING=UTF-8";
  putenv(python_ioencoding);

// On Windows add libCppController to Path (useful for C++ controllers, robot windows and remote control plugins)
// On macOS add libController to DYLD_LIBRARY_PATH (useful for generic robot window library)
#ifdef _WIN32
  const size_t new_path_size = snprintf(NULL, 0, "Path=%s\\msys64\\mingw64\\bin\\cpp;%s", WEBOTS_HOME, getenv("Path")) + 1;
  new_path = malloc(new_path_size);
  sprintf(new_path, "Path=%s\\msys64\\mingw64\\bin\\cpp;%s", WEBOTS_HOME, getenv("Path"));
  putenv(new_path);
#elif defined __APPLE__
  const size_t new_path_size =
    snprintf(NULL, 0, "DYLD_LIBRARY_PATH=%s/Contents/lib/controller:%s", WEBOTS_HOME, getenv("DYLD_LIBRARY_PATH")) + 1;
  new_path = malloc(new_path_size);
  sprintf(new_path, "DYLD_LIBRARY_PATH=%s/Contents/lib/controller:%s", WEBOTS_HOME, getenv("DYLD_LIBRARY_PATH"));
  putenv(new_path);
#endif
}

/*
This function sets the environment variable for MATLAB controllers execution.

- It sets WEBOTS_PROJECT environment variable to the path of the project folder.
- It sets WEBOTS_CONTROLLER_NAME environment variable to the name of the controller.
- It sets WEBOTS_CONTROLLER_ARGS environment variable to contain a list of all controller arguments.
- It determines the Webots version from version.txt file contained in the Webots installation, and sets
  WEBOTS_VERSION environment variable to this version.

The function also adds the controller library directory to the Path.
*/
static void matlab_config_environment() {
  // Add project folder to WEBOTS_PROJECT env variable
  get_current_path();
  const size_t controller_folder_size = strlen(strstr(current_path, "controllers")) + 1;
  const size_t controller_size = strlen(current_path);
  const size_t project_path_size = controller_size - controller_folder_size;
  char *project_path = malloc(project_path_size + 1);
  strncpy(project_path, current_path, project_path_size);
  project_path[project_path_size] = '\0';

  const size_t webots_project_size = snprintf(NULL, 0, "WEBOTS_PROJECT=%s", project_path) + 1;
  webots_project = malloc(webots_project_size);
  sprintf(webots_project, "WEBOTS_PROJECT=%s", project_path);
  putenv(webots_project);
  free(project_path);

  // Add controller name to WEBOTS_CONTROLLER_NAME env variable
  char *controller_name = strrchr(controller, PATH_SEPARATOR[0]) + 1;
  const size_t webots_controller_name_size = snprintf(NULL, 0, "WEBOTS_CONTROLLER_NAME=%s", controller_name) + 1;
  webots_controller_name = malloc(webots_controller_name_size);
  controller_name[strlen(controller_name) - strlen(controller_extension)] = '\0';
  sprintf(webots_controller_name, "WEBOTS_CONTROLLER_NAME=%s", controller_name);
  putenv(webots_controller_name);

  // Get Webots version (in txt file) and put it in WEBOTS_VERSION environment variable
#ifdef _WIN32
  const char *version_txt_path = "\\resources\\version.txt";
#elif defined __linux__
  const char *version_txt_path = "/resources/version.txt";
#elif defined __APPLE__
  const char *version_txt_path = "/Contents/Resources/version.txt";
#endif
  const size_t version_file_name_size = snprintf(NULL, 0, "%s%s", WEBOTS_HOME, version_txt_path) + 1;
  char *version_file_name = malloc(version_file_name_size);
  sprintf(version_file_name, "%s%s", WEBOTS_HOME, version_txt_path);

  FILE *version_file;
  if ((version_file = fopen(version_file_name, "r")) == NULL) {
    printf("Webots version could not be determined. '%s' can not be opened.\n", version_file_name);
    exit(1);
  }
  char version[16];  // RXXXXx-revisionX
  fscanf(version_file, "%15[^\n]", version);
  fclose(version_file);
  free(version_file_name);

  const size_t webots_version_size = snprintf(NULL, 0, "WEBOTS_VERSION=%s", version) + 1;
  webots_version = malloc(webots_version_size);
  sprintf(webots_version, "WEBOTS_VERSION=%s", version);
  putenv(webots_version);

  // Add libController to Path
  add_lib_controller_to_path();
}

// Replace all environment variables in the string ('$(ENV)' syntax) by its content
static void parse_environment_variables(char **string) {
  char *tmp = *string;
  while ((tmp = strstr(tmp, "$("))) {
    // Get environment variable string, name and length
    const char *end = strchr(tmp, ')');
    const size_t env_size = end - tmp + 1;
    char *var = malloc(env_size + 1);
    char *var_name = malloc(env_size - 2);
    strncpy(var, tmp, env_size);
    strncpy(var_name, tmp + 2, env_size - 3);
    var[env_size] = '\0';
    var_name[env_size - 3] = '\0';

    // Replace '$(ENV)' by the content in the complete string
    if (getenv(var_name))
      replace_substring(string, var, getenv(var_name));
    else
      replace_substring(string, var, "");

    tmp = *string;
    free(var);
    free(var_name);
  }
}

// Convert relative paths to absolute paths and replaces environment variables with their content in the ini file
static void format_ini_paths(char **string) {
  // Compute absolute path to ini file
  get_current_path();
  char *absolute_controller_path = NULL;
  size_t absolute_controller_path_size = 0;
  absolute_controller_path_size = snprintf(NULL, 0, "%s%s", current_path, controller_path) + 1;
  absolute_controller_path = malloc(absolute_controller_path_size);
  sprintf(absolute_controller_path, "%s%s", current_path, controller_path);

  // Add absolute path to runtime.ini in front of all relative paths
  char *tmp = strdup(*string);
  char *ptr = strtok(tmp, "=");
  int offset = 0;
  while (ptr != NULL) {
    int index = ptr - tmp + offset;
    if (index && ptr[0] != PATH_SEPARATOR[0] && ptr[0] != '$') {
      insert_string(string, absolute_controller_path, index);
      offset += absolute_controller_path_size - 1;
    }
    ptr = strtok(NULL, ENV_SEPARATOR);
  }
  free(absolute_controller_path);

  // Replace environment variables '$(ENV)' with their content
  parse_environment_variables(string);

  free(tmp);
}

// Read and apply the runtime.ini file line by line
static void parse_runtime_ini() {
  // Open runtime.ini if it exists
  const size_t ini_file_name_size = snprintf(NULL, 0, "%sruntime.ini", controller_path) + 1;
  char *ini_file_name = malloc(ini_file_name_size);
  sprintf(ini_file_name, "%sruntime.ini", controller_path);
  FILE *runtime_ini;
  if ((runtime_ini = fopen(ini_file_name, "r")) == NULL) {
    free(ini_file_name);
    return;
  }
  free(ini_file_name);

  // Read and process the runtime.ini file line by line
  char *line_buffer = malloc(MAX_LINE_BUFFER_SIZE);
  enum sections { Path, Simple, Windows, macOS, Linux } section;
  while (fgets(line_buffer, MAX_LINE_BUFFER_SIZE, runtime_ini)) {
    char *runtime_ini_line = malloc(strlen(line_buffer) + 1);
    strcpy(runtime_ini_line, line_buffer);
    remove_char(runtime_ini_line, ' ');   // remove useless spaces
    remove_char(runtime_ini_line, '\n');  // remove useless end-of-lines
    remove_char(runtime_ini_line, '\r');  // remove useless end-of-lines

    // Ignore empty lines
    if (!strlen(runtime_ini_line)) {
      free(runtime_ini_line);
      continue;
    }

    // Section line
    if (strncmp(runtime_ini_line, "[", 1) == 0) {
      if (strncmp(runtime_ini_line, "[environmentvariableswithpaths]", 31) == 0)
        section = Path;
      else if (strncmp(runtime_ini_line, "[environmentvariables]", 22) == 0)
        section = Simple;
      else if (strncmp(runtime_ini_line, "[environmentvariablesforWindows]", 32) == 0)
        section = Windows;
      else if (strncmp(runtime_ini_line, "[environmentvariablesformacOS]", 30) == 0)
        section = macOS;
      else if (strncmp(runtime_ini_line, "[environmentvariablesforLinux]", 30) == 0)
        section = Linux;
      else {
        fprintf(stderr,
                "Unknown section in the runtime.ini file. Please refer to "
                "https://cyberbotics.com/doc/guide/controller-programming#environment-variables for more information.\n");
        exit(1);
      }
    }
    // Commented line
    else if (strncmp(runtime_ini_line, ";", 1) == 0) {
      free(runtime_ini_line);
      continue;
    }
    // Key-value line
    else {
      char *new_env_ptr;
      switch (section) {
        case Path:
          remove_comment(runtime_ini_line);
#ifdef _WIN32
          // replace ':' and '/' by Windows equivalents
          replace_char(runtime_ini_line, '/', '\\');
          replace_char(runtime_ini_line, ':', ';');
#endif
          format_ini_paths(&runtime_ini_line);
          new_env_ptr = strdup(runtime_ini_line);
          putenv(new_env_ptr);
          break;
        case Simple:
          remove_comment(runtime_ini_line);
          new_env_ptr = strdup(runtime_ini_line);
          putenv(new_env_ptr);
          break;
        case Windows:
#ifdef _WIN32
          if (!strchr(runtime_ini_line, '\"')) {
            fprintf(stderr, "Windows paths should be written between double-quotes symbols \".\n");
            exit(1);
          }
          remove_comment(strrchr(runtime_ini_line, '\"'));
          remove_char(runtime_ini_line, '"');
          format_ini_paths(&runtime_ini_line);
          new_env_ptr = strdup(runtime_ini_line);
          putenv(new_env_ptr);
#endif
          break;
        case macOS:
#ifdef __APPLE__
          remove_comment(runtime_ini_line);
          format_ini_paths(&runtime_ini_line);
          new_env_ptr = strdup(runtime_ini_line);
          putenv(new_env_ptr);
#endif
          break;
        case Linux:
#ifdef __linux__
          remove_comment(runtime_ini_line);
          format_ini_paths(&runtime_ini_line);
          new_env_ptr = strdup(runtime_ini_line);
          putenv(new_env_ptr);
#endif
          break;
        default:
          break;
      }
    }
    free(runtime_ini_line);
  }
  free(line_buffer);
  fclose(runtime_ini);
}

// Add a single string argument to the argument 'argv' array
static char **add_single_argument(char **argv, size_t *current_size, char *str) {
  (*current_size)++;
  argv = realloc(argv, *current_size * sizeof(char *));
  if (!argv)
    exit(1);
  argv[(*current_size) - 1] = (str == NULL) ? NULL : strdup(str);
  return argv;
}

// Add all controller arguments (given to the launcher) to the 'argv' array
// For MATLAB, append them to WEBOTS_CONTROLLER_ARGS instead
static char **add_controller_arguments(char **argv, char **controller_argv, size_t *current_size, bool is_matlab) {
  size_t webots_controller_args_size;
  if (is_matlab) {
    webots_controller_args_size = snprintf(NULL, 0, "WEBOTS_CONTROLLER_ARGS=%s", controller_argv[next_argument_index]) + 1;
    webots_controller_args = malloc(webots_controller_args_size);
    snprintf(webots_controller_args, webots_controller_args_size, "WEBOTS_CONTROLLER_ARGS=%s",
             controller_argv[next_argument_index]);
    nb_controller_arguments--;
    next_argument_index++;
  }
  while (nb_controller_arguments) {
    if (is_matlab) {
      size_t varargin_size = snprintf(NULL, 0, "%s%s", ENV_SEPARATOR, controller_argv[next_argument_index]) + 1;
      char *varargin = malloc(varargin_size);
      snprintf(varargin, varargin_size, "%s%s", ENV_SEPARATOR, controller_argv[next_argument_index]);
      webots_controller_args = realloc(webots_controller_args, webots_controller_args_size + varargin_size);
      if (!webots_controller_args)
        exit(1);
      snprintf(webots_controller_args + webots_controller_args_size - 1, varargin_size, "%s", varargin);
      webots_controller_args_size += (varargin_size - 1);
      free(varargin);
    } else
      argv = add_single_argument(argv, current_size, controller_argv[next_argument_index]);
    nb_controller_arguments--;
    next_argument_index++;
  }
  if (is_matlab)
    putenv(webots_controller_args);
  return argv;
}

// Free allocated memory
static void free_memory() {
  free(WEBOTS_HOME);
  free(matlab_path);
  free(matlab_args);
  free(current_path);
  free(controller);

  free(WEBOTS_CONTROLLER_URL);
  free(new_path);
  free(new_windows_path);
  free(new_python_path);
  free(webots_project);
  free(webots_controller_name);
  free(webots_version);
  free(webots_controller_args);
}

int main(int argc, char **argv) {
  // Check WEBOTS_HOME and exit if empty
  if (!get_webots_home())
    exit(1);

  // Parse command line options
  if (!parse_options(argc, argv))
    exit(1);

#ifdef _WIN32
  // On Windows, convert forward slashes to backward slashes in controller path
  replace_char(controller, '/', '\\');
#endif

  // ROS 2 controller doesn't refer to a file but on the webots_ros2_driver package
  if (strcmp(controller, "ros2") == 0) {
    // Add the controller library to the Path.
    // Note: PYTHONPATH environment variable doesn't need to be set because it is already configured to point to the Python
    // installation folder of the lib controller when the ROS 2 package is sourced.
    add_lib_controller_to_path();
    size_t current_size = 0;
    char **new_argv = NULL;
    new_argv = add_single_argument(new_argv, &current_size, "ros2");
    new_argv = add_single_argument(new_argv, &current_size, "run");
    new_argv = add_single_argument(new_argv, &current_size, "webots_ros2_driver");
    new_argv = add_single_argument(new_argv, &current_size, "driver");
    new_argv = add_controller_arguments(new_argv, argv, &current_size, false);
    new_argv = add_single_argument(new_argv, &current_size, NULL);
#ifdef _WIN32
    const char *const *windows_argv = (const char **)new_argv;
    _spawnvpe(_P_WAIT, windows_argv[0], windows_argv, NULL);
#else
    execvp(new_argv[0], new_argv);
#endif
    free_memory();
    return 0;
  }

  // Check if controller file exists
  if (access(controller, F_OK) != 0) {
    fprintf(stderr, "Controller file '%s' not found. Please specify a path to an existing controller file.\n", controller);
    exit(1);
  }

  // Check if controller file is a directory
  struct stat path;
  stat(controller, &path);
  if (S_ISDIR(path.st_mode)) {
    fprintf(stderr, "Controller path '%s' is a directory. Please specify a path to an existing controller file.\n", controller);
    exit(1);
  };

  // Compute path to controller file
  char *controller_name =
    strrchr(controller, PATH_SEPARATOR[0]) == NULL ? NULL : strdup(strrchr(controller, PATH_SEPARATOR[0]));
  // Get extension from controller name (robust against relative paths)
  if (!controller_name)
    controller_extension = strrchr(controller, '.') == NULL ? NULL : strdup(strrchr(controller, '.'));
  else
    controller_extension = strrchr(controller_name, '.') == NULL ? NULL : strdup(strrchr(controller_name, '.'));
#ifdef _WIN32
  controller_path = strdup(".\\");
#else
  controller_path = strdup("./");
#endif
  if (strrchr(controller, PATH_SEPARATOR[0])) {
    const size_t controller_file_size = strlen(strrchr(controller, PATH_SEPARATOR[0])) - 1;
    const size_t controller_size = strlen(controller);
    const size_t controller_path_size = controller_size - controller_file_size;
    char *controller_path_tmp = malloc(controller_path_size + 1);
    strncpy(controller_path_tmp, controller, controller_path_size);
    controller_path_tmp[controller_path_size] = '\0';

    // Change to controller directory and edit controller file path
    chdir(controller_path_tmp);
    const size_t new_controller_size = snprintf(NULL, 0, "%s%s", controller_path, controller_name + 1) + 1;
    snprintf(controller, new_controller_size, "%s%s", controller_path, controller_name + 1);
    char *tmp_realloc = realloc(controller, new_controller_size);
    if (!tmp_realloc)
      exit(1);
    controller = tmp_realloc;
  } else {
    // Add current relative path to controller for execvp() function
    const size_t controller_size = strlen(controller);
    char *tmp_realloc = realloc(controller, controller_size + 3);
    if (!tmp_realloc)
      exit(1);
    controller = tmp_realloc;
    memmove(controller + 2, controller, controller_size + 1);
    memcpy(controller, controller_path, 2);
  }

  // Parse eventual runtime.ini file
  parse_runtime_ini();

  // Executable controller
  if (!controller_extension || strcmp(controller_extension, ".exe") == 0) {
    exec_java_config_environment();
    size_t current_size = 0;
    char **new_argv = NULL;
    new_argv = add_single_argument(new_argv, &current_size, controller);
    new_argv = add_controller_arguments(new_argv, argv, &current_size, false);
    new_argv = add_single_argument(new_argv, &current_size, NULL);
#ifdef _WIN32
    const char *const *windows_argv = (const char **)new_argv;
    _spawnvpe(_P_WAIT, windows_argv[0], windows_argv, NULL);
#else
    execvp(new_argv[0], new_argv);
#endif
  }
  // Python controller
  else if (strcmp(controller_extension, ".py") == 0) {
    python_config_environment();
    size_t current_size = 0;
    char **new_argv = NULL;
#ifdef _WIN32
    // Add quotation marks to the controller string if it contains whitespaces
    char *controller_formated = NULL;
    if (strstr(controller, " ") != NULL) {
      const size_t controller_formated_size = snprintf(NULL, 0, "\"%s\"", controller) + 1;
      controller_formated = malloc(controller_formated_size);
      sprintf(controller_formated, "\"%s\"", controller);
    } else
      controller_formated = strdup(controller);
    new_argv = add_single_argument(new_argv, &current_size, "python");
    new_argv = add_single_argument(new_argv, &current_size, "-u");
    new_argv = add_single_argument(new_argv, &current_size, controller_formated);
#else
    new_argv = add_single_argument(new_argv, &current_size, "python3");
    new_argv = add_single_argument(new_argv, &current_size, "-u");
    new_argv = add_single_argument(new_argv, &current_size, controller);
#endif
    new_argv = add_controller_arguments(new_argv, argv, &current_size, false);
    new_argv = add_single_argument(new_argv, &current_size, NULL);
#ifdef _WIN32
    const char *const *windows_argv = (const char **)new_argv;
    _spawnvpe(_P_WAIT, windows_argv[0], windows_argv, NULL);
#else
    execvp(new_argv[0], new_argv);
#endif
  }
  // MATLAB controller
  else if (strcmp(controller_extension, ".m") == 0) {
    matlab_config_environment();

    // If no MATLAB installation path was given in command line, check in default installation folder
    if (!matlab_path && !get_matlab_path())
      return -1;

    if (!matlab_args) {
      matlab_args = malloc(strlen("-batch") + 1);
      sprintf(matlab_args, "-batch");
    } else {
      printf("Running MATLAB in interactive mode...\n");
    }

#ifdef _WIN32
    const char *launcher_path = "\\lib\\controller\\matlab";
#elif defined __APPLE__
    const char *launcher_path = "/Contents/lib/controller/matlab";
#elif defined __linux__
    const char *launcher_path = "/lib/controller/matlab";
#endif
    // matlab_command starts the launcher.m file contained in the lib controller
    const size_t matlab_command_size = snprintf(NULL, 0, "-sd \"%s%s\"", WEBOTS_HOME, launcher_path) + 1;
    char *matlab_command = malloc(matlab_command_size);
    sprintf(matlab_command, "-sd \"%s%s\"", WEBOTS_HOME, launcher_path);

    // Start MATLAB without display and run the launcher.m file
    size_t current_size = 0;
    char **new_argv = NULL;
    new_argv = add_single_argument(new_argv, &current_size, matlab_path);
    new_argv = add_single_argument(new_argv, &current_size, matlab_command);
    new_argv = add_single_argument(new_argv, &current_size, matlab_args);
    new_argv = add_single_argument(new_argv, &current_size, "launcher");
    if (nb_controller_arguments)
      new_argv = add_controller_arguments(new_argv, argv, &current_size, true);
    new_argv = add_single_argument(new_argv, &current_size, NULL);

#ifdef _WIN32
    const char *const *windows_argv = (const char **)new_argv;
    _spawnvpe(_P_WAIT, windows_argv[0], windows_argv, NULL);
#else
    execvp(new_argv[0], new_argv);
#endif
    free(matlab_command);
  }
  // Java controller
  else if (strcmp(controller_extension, ".jar") == 0 || strcmp(controller_extension, ".class") == 0) {
    exec_java_config_environment();

    // Write path to java lib controller
#ifdef _WIN32
    const char *java_lib_controller = "\\lib\\controller\\java";
#elif defined __APPLE__
    const char *java_lib_controller = "/Contents/lib/controller/java";
#elif defined __linux__
    const char *java_lib_controller = "/lib/controller/java";
#endif
    const size_t lib_controller_size = snprintf(NULL, 0, "%s%s", WEBOTS_HOME, java_lib_controller) + 1;
    char *lib_controller = malloc(lib_controller_size);
    sprintf(lib_controller, "%s%s", WEBOTS_HOME, java_lib_controller);

#ifdef _WIN32
    const char *jar_path = "\\Controller.jar;";
#else
    const char *jar_path = "/Controller.jar:";
#endif
    char *short_controller_path = strdup(controller_path);
    short_controller_path[strlen(controller_path) - 1] = '\0';
#ifdef _WIN32
    // Write the 'classpath' option (mandatory for java controllers) with the path to Controller.jar and the path to the
    // controller
    const size_t classpath_size = snprintf(NULL, 0, "\"%s%s%s\"", lib_controller, jar_path, short_controller_path) + 1;
    char *classpath = malloc(classpath_size);
    sprintf(classpath, "\"%s%s%s\"", lib_controller, jar_path, short_controller_path);

    // Write the '-Djava.library.path' option (mandatory for java controllers) with the path to the libController
    const size_t java_library_size = snprintf(NULL, 0, "\"-Djava.library.path=%s\"", lib_controller) + 1;
    char *java_library = malloc(java_library_size);
    sprintf(java_library, "\"-Djava.library.path=%s\"", lib_controller);
#else
    // Write the 'classpath' option (mandatory for java controllers) with the path to Controller.jar and the path to the
    // controller
    const size_t classpath_size = snprintf(NULL, 0, "%s%s%s", lib_controller, jar_path, short_controller_path) + 1;
    char *classpath = malloc(classpath_size);
    sprintf(classpath, "%s%s%s", lib_controller, jar_path, short_controller_path);

    // Write the '-Djava.library.path' option (mandatory for java controllers) with the path to the libController
    const size_t java_library_size = snprintf(NULL, 0, "-Djava.library.path=%s", lib_controller) + 1;
    char *java_library = malloc(java_library_size);
    sprintf(java_library, "-Djava.library.path=%s", lib_controller);
#endif
    if (!controller_name)
      controller_name = strrchr(controller, PATH_SEPARATOR[0]);
    controller_name[strlen(controller_name) - strlen(controller_extension)] = '\0';
    size_t current_size = 0;
    char **new_argv = NULL;
    // Java command has the following syntax: 'java -classpath [...] -Djava.library.path=[...] controller_name'
    new_argv = add_single_argument(new_argv, &current_size, "java");
    new_argv = add_single_argument(new_argv, &current_size, "-classpath");
    new_argv = add_single_argument(new_argv, &current_size, classpath);
    new_argv = add_single_argument(new_argv, &current_size, java_library);
    new_argv = add_single_argument(new_argv, &current_size, controller_name + 1);
    new_argv = add_controller_arguments(new_argv, argv, &current_size, false);
    new_argv = add_single_argument(new_argv, &current_size, NULL);
#ifdef _WIN32
    const char *const *windows_argv = (const char **)new_argv;
    _spawnvpe(_P_WAIT, windows_argv[0], windows_argv, NULL);
#else
    execvp(new_argv[0], new_argv);
#endif

    free(lib_controller);
    free(short_controller_path);
    free(classpath);
    free(java_library);
  } else
    fprintf(stderr,
            "The file extension '%s' is not supported as webots controller. Supported file types are executable files, '.py', "
            "'.jar', '.class' and '.m'.\n",
            controller_extension);

  free_memory();
  return 0;
}
