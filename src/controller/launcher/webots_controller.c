/*
 * Copyright 1996-2023 Cyberbotics Ltd.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <dirent.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#ifdef WIN32
#include <io.h>
#define F_OK 0
#define access _access
#endif

char *WEBOTS_HOME;
char *controller;
char *controller_path;
char *controller;
char *controller_extension;
char *matlab_path;
char *current_path;

void get_current_path() {
  if (!current_path) {
    current_path = malloc(512);
    getcwd(current_path, 512);
#ifdef _WIN32
    strcat(current_path, "\\");
#else
    strcat(current_path, "/");
#endif
    printf("%s\n", current_path);
  }
}

bool get_webots_home() {
  if (!getenv("WEBOTS_HOME")) {
    printf("Set the path to your webots installation folder in WEBOTS_HOME environment variable.\n");
    return false;
  } else
    WEBOTS_HOME = malloc(strlen(getenv("WEBOTS_HOME")) + 1);
  strcpy(WEBOTS_HOME, getenv("WEBOTS_HOME"));
  return true;
}

bool get_matlab_path() {
  struct dirent *directory_entry;  // Pointer for directory entry

#ifdef __APPLE__
  const char *matlab_directory = "/Applications/";
  const char *matlab_version_wc = "MATLAB_R20";
#else
  const char *matlab_version_wc = "R20";
#ifdef _WIN32
  const char *matlab_directory = "C:\\Program Files\\MATLAB\\";
  const char *matlab_exec_suffix = "\\bin\\win64\\MATLAB.exe";
#else  // __linux__
  const char *matlab_directory = "/usr/local/MATLAB/";
  // cppcheck-suppress unreadVariable
  const char *matlab_exec_suffix = "/bin/matlab";
#endif
#endif

  DIR *directory = opendir(matlab_directory);
#ifndef __APPLE__
  if (directory == NULL) {
    printf("No installation of Matlab available.\n");
    return false;
  }
#endif
  // Get latest available Matlab version
  char *latest_version = NULL;
  while ((directory_entry = readdir(directory)) != NULL) {
    if (strncmp(matlab_version_wc, directory_entry->d_name, strlen(matlab_version_wc)) == 0) {
      size_t directory_name_size = strlen(directory_entry->d_name);
      if (!latest_version) {
        latest_version = malloc(directory_name_size);
        strncpy(latest_version, directory_entry->d_name, directory_name_size);
      }
      if (latest_version && strcmp(latest_version, directory_entry->d_name) < 0) {
        memset(latest_version, '\0', sizeof(latest_version));
        strncpy(latest_version, directory_entry->d_name, directory_name_size);
      }
    }
  }
  // printf("%s\n", latest_version);
#ifdef __APPLE__
  if (latest_version == NULL) {
    printf("No installation of Matlab available.\n");
    return false;
  }
#endif
  closedir(directory);

#ifdef __APPLE__
  size_t matlab_path_size = snprintf(NULL, 0, "%s%s", matlab_directory, latest_version) + 1;
  matlab_path = malloc(matlab_path_size);
  sprintf(matlab_path, "%s%s", matlab_directory, latest_version);
#else
  size_t matlab_path_size = snprintf(NULL, 0, "%s%s%s", matlab_directory, latest_version, matlab_exec_suffix) + 1;
  matlab_path = malloc(matlab_path_size);
  sprintf(matlab_path, "%s%s%s", matlab_directory, latest_version, matlab_exec_suffix);
#endif

  return true;
}

void print_options() {
  printf(
    "Usage: webots-controller [options] [controller_file]\n\nOptions:\n\n  --protocol=<ipc|tcp>\n    ipc is used by "
    "default. ipc should be used when Webots is running on the same machine as the extern controller. tcp should be used when "
    "connecting to a remote instance of Webots.\n\n  --ip-address=<ip-address>\n    The IP address of the remote machine on "
    "which the Webots instance is running. This option should only be used with the tcp protocol (remote controllers).\n\n  "
    "--port=<port>\n    1234 is used by default, as it is the default port of Webots. This parameter allows to connect to a "
    "specific instance of Webots if multiple of them are running. The port of a Webots instance can be set at its launch.\n\n  "
    "--robot-name=<robot-name>\n    Target a specific robot by specifiyng its name in case multiple robots wait for an extern "
    "controller in the Webots instance.\n\n  --matlab-path=<matlab-path>\n    For MATLAB controllers, this option allows to "
    "specify the path to the executable of a specific MATLAB version. By default, the launcher checks in the default MATLAB "
    "installation folder. See https://cyberbotics.com/doc/guide/using-matlab#matlab-installation for more information.\n\n  "
    "--stdout-redirect>\n    Redirect the stdout of the controller to the terminal of Webots.\n\n  --stderr-redirect>\n    "
    "Redirect the stderr of the controller to the terminal of Webots.\n\n");
}

bool parse_options(int nb_arguments, char **arguments) {
  if (nb_arguments == 1) {
    printf("No controller file provided. Please provide an existing controller file as argument.\n");
    return false;
  }

  controller = NULL;
  matlab_path = NULL;
  char *protocol = NULL;
  char *ip_address = NULL;
  char *port = NULL;
  char *robot_name = NULL;
  for (int i = 1; i < nb_arguments; i++) {
    if (arguments[i][0] == '-') {
      if (strncmp(arguments[i] + 2, "protocol=", 9) == 0) {
        size_t protocol_size = strlen(arguments[i] + 11) + 1;
        protocol = malloc(protocol_size);
        memcpy(protocol, arguments[i] + 11, protocol_size);
        // printf("protocol = %s\n", protocol);
      } else if (strncmp(arguments[i] + 2, "ip-address=", 11) == 0) {
        size_t ip_address_size = strlen(arguments[i] + 13) + 1;
        ip_address = malloc(ip_address_size);
        memcpy(ip_address, arguments[i] + 13, ip_address_size);
        // printf("ip_address = %s\n", ip_address);
      } else if (strncmp(arguments[i] + 2, "port=", 5) == 0) {
        size_t port_size = strlen(arguments[i] + 7) + 1;
        port = malloc(port_size);
        memcpy(port, arguments[i] + 7, port_size);
        // printf("port = %s\n", port);
      } else if (strncmp(arguments[i] + 2, "robot-name=", 11) == 0) {
        size_t robot_name_size = strlen(arguments[i] + 13) + 1;
        robot_name = malloc(robot_name_size);
        memcpy(robot_name, arguments[i] + 13, robot_name_size);
        // printf("robot_name = %s\n", robot_name);
      } else if (strncmp(arguments[i] + 2, "matlab-path=", 12) == 0) {
        size_t matlab_path_size = strlen(arguments[i] + 14) + 1;
        matlab_path = malloc(matlab_path_size);
        memcpy(matlab_path, arguments[i] + 14, matlab_path_size);
        // printf("matlab_path = %s\n", matlab_path);
      } else if (strncmp(arguments[i] + 2, "stdout-redirect", 15) == 0) {
        putenv("WEBOTS_STDOUT_REDIRECT=1");
      } else if (strncmp(arguments[i] + 2, "stderr-redirect", 15) == 0) {
        putenv("WEBOTS_STDERR_REDIRECT=1");
      } else if (strncmp(arguments[i] + 2, "help", 4) == 0) {
        print_options();
        return false;
      } else {
        printf("Invalid option '%s'. Try '--help' for more information.\n", arguments[i]);
        return false;
      }
    } else {
      if (controller) {
        printf("Please specify only one single controller file to launch.\n");
        return false;
      }
      size_t controller_size = strlen(arguments[i]) + 1;
      controller = malloc(controller_size);
      memcpy(controller, arguments[i], controller_size);
    }
  }

  // Check that a controller path has been provided
  if (!controller) {
    printf("No controller file provided. Please provide an existing controller file as argument.\n");
    return false;
  }

  // If no protocol is given, ipc is used by default
  if (!protocol)
    protocol = strdup("ipc");

  // If no port is given, 1234 is used by default
  if (!port)
    port = strdup("1234");

  char *WEBOTS_CONTROLLER_URL = NULL;
  // Write WEBOTS_CONTROLLER_URL in function of given options
  if (strncmp(protocol, "tcp", 3) == 0) {
    if (!ip_address) {
      printf("Specify the IP address of the Webots machine to connect to with '--ip_address=' option.\n");
      return false;
    }

    size_t WEBOTS_CONTROLLER_URL_size =
      snprintf(NULL, 0, "%s%s%s%s%s%s", "WEBOTS_CONTROLLER_URL=", protocol, "://", ip_address, ":", port) + 1;
    WEBOTS_CONTROLLER_URL = malloc(WEBOTS_CONTROLLER_URL_size);
    sprintf(WEBOTS_CONTROLLER_URL, "%s%s%s%s%s%s", "WEBOTS_CONTROLLER_URL=", protocol, "://", ip_address, ":", port);
  } else if (strncmp(protocol, "ipc", 3) == 0) {
    if (ip_address)
      printf("Skipping IP address for ipc protocol.\n");

    size_t WEBOTS_CONTROLLER_URL_size = snprintf(NULL, 0, "%s%s%s%s", "WEBOTS_CONTROLLER_URL=", protocol, "://", port) + 1;
    WEBOTS_CONTROLLER_URL = malloc(WEBOTS_CONTROLLER_URL_size);
    sprintf(WEBOTS_CONTROLLER_URL, "%s%s%s%s", "WEBOTS_CONTROLLER_URL=", protocol, "://", port);
  } else {
    printf("Only ipc and tcp protocols are supported.\n");
    return false;
  }

  // If a robot name is specified, add it to WEBOTS_CONTROLLER_URL
  if (robot_name) {
    size_t with_robot_size = snprintf(NULL, 0, "%s%s%s", WEBOTS_CONTROLLER_URL, "/", robot_name) + 1;
    WEBOTS_CONTROLLER_URL = realloc(WEBOTS_CONTROLLER_URL, with_robot_size);
    sprintf(WEBOTS_CONTROLLER_URL, "%s%s%s", WEBOTS_CONTROLLER_URL, "/", robot_name);
  }
  putenv(WEBOTS_CONTROLLER_URL);

  // Show resulting target options to user
  const char *location = strncmp(protocol, "tcp", 3) == 0 ? "remote" : "local";
  printf("The started controller targets a %s instance (%s protocol) of Webots with port number %s.", location, protocol, port);
  strncmp(protocol, "tcp", 3) == 0 ? printf(" The IP address of the remote Webots instance is '%s'. ", ip_address) :
                                     printf(" ");
  robot_name ? printf("Targeting robot '%s'.\n\n", robot_name) :
               printf("Targeting the only robot waiting for an extern controller.\n\n");
  // printf("%s\n", getenv("WEBOTS_CONTROLLER_URL"));

  return true;
}

void exec_java_config_environment() {
#ifdef _WIN32
  size_t new_path_size =
    snprintf(NULL, 0, "%s%s%s%s%s%s%s%s", "Path=", WEBOTS_HOME, "\\lib\\controller;", WEBOTS_HOME, "\\msys64\\mingw64\\bin;",
             WEBOTS_HOME, "\\msys64\\mingw64\\bin\\cpp;", getenv("DYLD_LIBRARY_PATH")) +
    1;
  char *new_path = malloc(new_path_size);
  sprintf(new_path, "%s%s%s%s%s%s%s%s", "Path=", WEBOTS_HOME, "\\lib\\controller;", WEBOTS_HOME, "\\msys64\\mingw64\\bin;",
          WEBOTS_HOME, "\\msys64\\mingw64\\bin\\cpp;", getenv("DYLD_LIBRARY_PATH"));
  putenv(new_path);
#else
#ifdef __linux__
  const char *lib_controller = "/lib/controller:";
  const char *ld_env_variable = "LD_LIBRARY_PATH";
#else  //__APPLE__
  const char *lib_controller = "/Contents/lib/controller:";
  const char *ld_env_variable = "DYLD_LIBRARY_PATH";
#endif
  size_t new_ld_path_size =
    snprintf(NULL, 0, "%s%s%s%s%s", ld_env_variable, "=", WEBOTS_HOME, lib_controller, getenv(ld_env_variable)) + 1;
  char *new_ld_path = malloc(new_ld_path_size);
  sprintf(new_ld_path, "%s%s%s%s%s", ld_env_variable, "=", WEBOTS_HOME, lib_controller, getenv(ld_env_variable));
  putenv(new_ld_path);
#endif
}

void python_config_environment() {
#ifdef _WIN32
  const char *python_lib_controller = "\\lib\\controller\\python:";
#elif defined __linux__
  const char *python_lib_controller = "/lib/controller/python:";
#elif defined __APPLE__
  const char *python_lib_controller = "/Contents/lib/controller/python:";
#endif
  size_t new_python_path_size =
    snprintf(NULL, 0, "%s%s%s%s", "PYTHONPATH=", WEBOTS_HOME, python_lib_controller, getenv("PYTHONPATH")) + 1;
  char *new_python_path = malloc(new_python_path_size);
  sprintf(new_python_path, "%s%s%s%s", "PYTHONPATH=", WEBOTS_HOME, python_lib_controller, getenv("PYTHONPATH"));
  putenv(new_python_path);

  char *python_ioencoding = "PYTHONIOENCODING=UTF-8";
  putenv(python_ioencoding);

  // on windows add the bin/cpp to Path for e-puck
}

void matlab_config_environment() {
  // Add project folder to WEBOTS_PROJECT env variable
  const size_t controller_folder_size = strlen(strstr(controller, "controllers")) + 1;
  const size_t controller_size = strlen(controller);
  const size_t project_path_size = controller_size - controller_folder_size;
  char *project_path = malloc(project_path_size + 1);
  strncpy(project_path, controller, project_path_size);
  project_path[project_path_size] = '\0';

  get_current_path();
  size_t webots_project_size = snprintf(NULL, 0, "%s%s%s", "WEBOTS_PROJECT=", current_path, project_path) + 1;
  char *webots_project = malloc(webots_project_size);
  sprintf(webots_project, "%s%s%s", "WEBOTS_PROJECT=", current_path, project_path);
  putenv(webots_project);

  // Add controller name to WEBOTS_CONTROLLER_NAME env variable
#ifdef _WIN32
  char *controller_name = strrchr(controller, '\\') + 1;
#else
  char *controller_name = strrchr(controller, '/') + 1;
#endif
  size_t webots_controller_name_size = snprintf(NULL, 0, "%s%s", "WEBOTS_CONTROLLER_NAME=", controller_name) + 1;
  char *webots_controller_name = malloc(webots_controller_name_size);
  controller_name[strlen(controller_name) - strlen(controller_extension)] = '\0';
  sprintf(webots_controller_name, "%s%s", "WEBOTS_CONTROLLER_NAME=", controller_name);
  putenv(webots_controller_name);

  // Get Webots version and put it in WEBOTS_VERSION env variable
#ifdef _WIN32
  const char *version_txt_path = "\\resources\\version.txt";
#else
  const char *version_txt_path = "/resources/version.txt";
#endif
  size_t version_file_name_size = snprintf(NULL, 0, "%s%s", WEBOTS_HOME, version_txt_path) + 1;
  char *version_file_name = malloc(version_file_name_size);
  sprintf(version_file_name, "%s%s", WEBOTS_HOME, version_txt_path);

  FILE *version_file;
  if ((version_file = fopen(version_file_name, "r")) == NULL) {
    printf("Webots version could not be determined. '%s' can not be opened.\n", version_file_name);
    exit(1);
  }
  char version[16];  // RXXXXx-revisionX
  fscanf(version_file, "%[^\n]", version);
  fclose(version_file);

  size_t webots_version_size = snprintf(NULL, 0, "%s%s", "WEBOTS_VERSION=", version) + 1;
  char *webots_version = malloc(webots_version_size);
  sprintf(webots_version, "%s%s", "WEBOTS_VERSION=", version);
  putenv(webots_version);
}

void remove_comment(char *string) {
  const char *comment = strchr(string, ';');
  if (comment) {
    const size_t comment_size = strlen(comment);
    const size_t full_line_size = strlen(string);
    const size_t content_size = full_line_size - comment_size;
    string[content_size] = '\0';
    return;
  }
  string[strlen(string) - 1] = '\0';
}

void replace_char(char *string, char occurence, char replace) {
  char *current_pos = strchr(string, occurence);
  while (current_pos) {
    *current_pos = replace;
    current_pos = strchr(current_pos + 1, occurence);
  }
}

void remove_char(char *string, char occurence) {
  char *removed = string;
  do {
    while (*removed == occurence) {
      ++removed;
    }
  } while (*string++ = *removed++);
}

void insert_string_at_index(char **string, char *insert, int index) {
  size_t new_size = strlen(*string) + strlen(insert) + 1;

  char *tmp = strdup(*string);
  *string = realloc(*string, new_size);
  strncpy(*string + index, insert, strlen(insert));
  strncpy(*string + index + strlen(insert), tmp + index, strlen(tmp) - index);
  (*string)[new_size - 1] = '\0';
}

void parse_ini_paths(char **string) {
  // Compute absolute path to ini file
  get_current_path();
  size_t absolute_controller_path_size = snprintf(NULL, 0, "%s%s", current_path, controller_path) + 1;
  char *absolute_controller_path = malloc(absolute_controller_path_size);
  sprintf(absolute_controller_path, "%s%s", current_path, controller_path);

  // Add absolute path to runtime.ini in front of all relative paths
  char *tmp = strdup(*string);
  char *ptr = strtok(tmp, "=");
  char *env_name = ptr;
  int offset = 0;
  while (ptr != NULL) {
    int index = ptr - tmp + offset;
    // printf("ptr = %s\n", ptr);
#ifdef _WIN32
    if (index && ptr[0] != '\\' && ptr[0] != '$') {
      insert_string_at_index(*string, absolute_controller_path, index);
      offset += absolute_controller_path_size - 1;
    }
    ptr = strtok(NULL, ";");
#else
    if (index && ptr[0] != '/' && ptr[0] != '$') {
      insert_string_at_index(string, absolute_controller_path, index);
      offset += absolute_controller_path_size - 1;
    }
    ptr = strtok(NULL, ":");
#endif
  }

  // Append previous environment string
  if (getenv(env_name)) {
    size_t new_size = snprintf(NULL, 0, "%s:%s", *string, getenv(env_name)) + 1;
    *string = realloc(*string, new_size);
#ifdef _WIN32
    sprintf(*string, "%s;%s", *string, getenv(env_name));
#else
    sprintf(*string, "%s:%s", *string, getenv(env_name));
#endif
  }

  // TODO: replace $() by actual value
  /*char *variable = strstr(string, "$(");
  if (variable) {
    char *variable_end = strchr(variable, ')');
    char *test = malloc(100);
    strncpy(test, variable, variable_end - variable);
  }*/

  printf("%s\n", *string);
}

void parse_runtime_ini() {
  // Compute path to controller file
#ifdef _WIN32
  const size_t controller_file_size = strlen(strrchr(controller, '\\')) - 1;
#else
  const size_t controller_file_size = strlen(strrchr(controller, '/')) - 1;
#endif
  const size_t controller_size = strlen(controller);
  const size_t controller_path_size = controller_size - controller_file_size;
  controller_path = malloc(controller_path_size + 1);
  strncpy(controller_path, controller, controller_path_size);
  controller_path[controller_path_size] = '\0';

  // Open runtime.ini if it exists
  size_t ini_file_name_size = snprintf(NULL, 0, "%s%s", controller_path, "/runtime.ini") + 1;
  char *ini_file_name = malloc(ini_file_name_size);
  sprintf(ini_file_name, "%s%s", controller_path, "/runtime.ini");
  FILE *runtime_ini;
  if ((runtime_ini = fopen(ini_file_name, "r")) == NULL)
    return;

  // Read the file line by line
  ssize_t line_size;
  size_t buffer_size;
  char *line = NULL;
  enum sections { Path, Simple, Windows, macOS, Linux } section;
  while ((line_size = getline(&line, &buffer_size, runtime_ini)) != -1) {
    // printf("%s", line);
    remove_char(line, ' ');   // remove useless spaces
    remove_char(line, '\n');  // remove useless end-of-lines
    // printf("%s", line);
    line_size = strlen(line) - 1;  // re-evaluate line size and ignore end-of-line
    // printf("%d\n", line_size);
    // Empty line
    if (line_size <= 1)
      continue;

    // Section line
    if (strncmp(line, "[", 1) == 0) {
      if (strncmp(line, "[environmentvariableswithpaths]", 31) == 0) {
        section = Path;
        // printf("0: %s", line);
      } else if (strncmp(line, "[environmentvariables]", 22) == 0) {
        section = Simple;
        // printf("1: %s", line);
      } else if (strncmp(line, "[environmentvariablesforWindows]", 32) == 0) {
        section = Windows;
        // printf("2: %s", line);
      } else if (strncmp(line, "[environmentvariablesformacOS]", 30) == 0) {
        section = macOS;
        // printf("3: %s", line);
      } else if (strncmp(line, "[environmentvariablesforLinux]", 30) == 0) {
        section = Linux;
        // printf("4: %s", line);
      } else {
        printf("Unknown section in the runtime.ini file. Please refer to "
               "https://cyberbotics.com/doc/guide/controller-programming#environment-variables for more information.\n");
        exit(1);
      }
    }
    // Key-value line
    else {
      switch (section) {
        case Path:
          remove_comment(line);
          line_size = strlen(line);  // Update line size

#ifdef _WIN32
          // replace ':' and '/' by Windows equivalents
          replace_char(line, '/', '\\');
          replace_char(line, ':', ';');
#endif

          parse_ini_paths(&line);
          /*char *value = strchr(line, '=') + 1;
          const size_t value_size = strlen(value);
          const size_t key_size = line_size - value_size + 1;
          char *key = malloc(key_size + 1);
          strncpy(key, line, key_size);
          key[key_size] = '\0';
          printf("%s and %s\n", key, value);*/
          // line[strlen(line) - 1] = '\0';
          putenv(line);

          break;
        case Simple:
          remove_comment(line);
          line_size = strlen(line);  // Update line size
          putenv(line);
          break;
        case Windows:
          if (!strchr(line, '\"')) {
            printf("Paths for windows should be written between double-quotes symbols \".\n");
            exit(1);
          }
          remove_comment(strrchr(line, '\"'));
          // printf("%s\n", line);
#ifdef _WIN32
          remove_char(line, '"');
          parse_ini_paths(&line);
          putenv(line);
#endif
          break;
        case macOS:
#ifdef __APPLE__
          remove_comment(line);
          line_size = strlen(line);  // Update line size
          parse_ini_paths(&line);
          putenv(line);
#endif
          break;
        case Linux:
#ifdef __linux__
          remove_comment(line);
          line_size = strlen(line);  // Update line size
          parse_ini_paths(&line);
          putenv(line);
#endif
          break;
      }
    }
  }
  fclose(runtime_ini);
}

int main(int argc, char **argv) {
  // Check WEBOTS_HOME and exit if empty
  const bool is_set = get_webots_home();
  if (!is_set)
    return -1;

  // Parse command line options
  const bool success = parse_options(argc, argv);
  if (!success)
    return -1;

  // Parse possible runtime.ini file
  parse_runtime_ini();

  // Check if controller file exists
  if (access(controller, F_OK) != 0) {
    printf("Controller file '%s' not found. Please specify a path to an existing controller file.\n", controller);
    return -1;
  }

  // Get extension from controller name (robust against relative paths)
#ifdef _WIN32
  char *controller_name = strrchr(controller, '\\');
#else
  char *controller_name = strrchr(controller, '/');
#endif
  if (!controller_name)
    controller_extension = strrchr(controller, '.');
  else
    controller_extension = strrchr(controller_name, '.');

  // Executable controller
  if (!controller_extension) {
    exec_java_config_environment();
    system(controller);
  }
  // Python controller
  else if (strcmp(controller_extension, ".py") == 0) {
    python_config_environment();

#ifdef _WIN32
    const char *python_prefix = "python ";
#else
    const char *python_prefix = "python3 ";
#endif
    size_t python_command_size = snprintf(NULL, 0, "%s%s", python_prefix, controller) + 1;
    char *python_command = malloc(python_command_size);
    sprintf(python_command, "%s%s", python_prefix, controller);

    // printf("%s\n", python_command);
    system(python_command);
  }
  // Matlab controller
  else if (strcmp(controller_extension, ".m") == 0) {
    matlab_config_environment();

    // If no Matlab path was given in command line, check in default installation folder
    if (!matlab_path) {
      const bool default_matlab_install = get_matlab_path();
      // printf("%s\n", matlab_path);
      if (!default_matlab_install)
        return -1;
    }

#ifdef _WIN32
    const char *launcher_path = "\\lib\\controller\\matlab\\launcher.m'); exit;\"";
#elif defined __APPLE__
    const char *launcher_path = "/Contents/lib/controller/matlab/launcher.m'); exit;\"";
#elif defined __linux__
    const char *launcher_path = "/lib/controller/matlab/launcher.m'); exit;\"";
#endif

    size_t matlab_command_size =
      snprintf(NULL, 0, "%s%s%s%s", matlab_path, " -nodisplay -nosplash -nodesktop -r \"run('", WEBOTS_HOME, launcher_path) + 1;
    char *matlab_command = malloc(matlab_command_size);
    sprintf(matlab_command, "%s%s%s%s", matlab_path, " -nodisplay -nosplash -nodesktop -r \"run('", WEBOTS_HOME, launcher_path);

    // printf("%s\n", matlab_command);
    system(matlab_command);
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
    size_t lib_controller_size = snprintf(NULL, 0, "%s%s", WEBOTS_HOME, java_lib_controller) + 1;
    char *lib_controller = malloc(lib_controller_size);
    sprintf(lib_controller, "%s%s", WEBOTS_HOME, java_lib_controller);

    // Write the 'classpath' option (mandatory for java controllers)
#ifdef _WIN32
    const char *jar_path = "\\Controller.jar:";
#else
    const char *jar_path = "/Controller.jar:";
#endif
    size_t classpath_size = snprintf(NULL, 0, "%s%s%s%s", "-classpath ", lib_controller, jar_path, controller_path) + 1;
    char *classpath = malloc(classpath_size);
    sprintf(classpath, "%s%s%s%s", "-classpath ", lib_controller, jar_path, controller_path);

    // Write the 'Djava.library.path' option (mandatory for java controllers)
    size_t java_library_size = snprintf(NULL, 0, "%s%s", " -Djava.library.path=", lib_controller) + 1;
    char *java_library = malloc(java_library_size);
    sprintf(java_library, "%s%s", " -Djava.library.path=", lib_controller);

    // Write arguments to java command
    controller_name[strlen(controller_name) - strlen(controller_extension)] = '\0';
    size_t java_command_size = snprintf(NULL, 0, "%s%s%s%s%s", "java ", classpath, java_library, " ", controller_name + 1) + 1;
    char *java_command = malloc(java_command_size);
    sprintf(java_command, "%s%s%s%s%s", "java ", classpath, java_library, " ", controller_name + 1);

    // printf("%s\n", java_command);
    system(java_command);
  } else
    printf("The file extension '%s' is not supported as webots controller. Supported file types are executables, '.py', "
           "'.jar', '.class' and '.m'.\n",
           controller_extension);

  return 0;
}
