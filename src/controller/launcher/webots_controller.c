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
char *matlab_path;

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
    "Usage: webots_controller [options] [controller_file]\n\nOptions:\n\n  --protocol=<ipc|tcp>\n    ipc is used by "
    "default. ipc should be used when Webots is running on the same machine as the extern controller. tcp should be used when "
    "connecting to a remote instance of Webots.\n\n  --ip_address=<ip_address>\n    The IP address of the remote machine on "
    "which the Webots instance is running. This option should only be used with the tcp protocol (remote controllers).\n\n  "
    "--port=<port>\n    1234 is used by default, as it is the default port of Webots. This parameter allows to connect to a "
    "specific instance of Webots if multiple of them are running. The port of a Webots instance can be set at its launch.\n\n  "
    "--robot_name=<robot_name>\n    Target a specific robot by specifiyng its name in case multiple robots wait for an extern "
    "controller in the Webots instance.\n");
}

bool parse_options(int nb_arguments, char **arguments) {
  if (nb_arguments == 1) {
    printf("No controller file provided. Please provide an existing controller file as argument.\n");
    return false;
  }

  char *protocol = NULL;
  size_t protocol_size = 0;
  char *ip_address = NULL;
  size_t ip_address_size = 0;
  char *port = NULL;
  size_t port_size = 0;
  char *robot_name = NULL;
  size_t robot_name_size = 0;
  controller = NULL;
  for (int i = 1; i < nb_arguments; i++) {
    if (arguments[i][0] == '-') {
      if (strncmp(arguments[i] + 2, "protocol=", 9) == 0) {
        protocol_size = strlen(arguments[i] + 11) + 1;
        protocol = malloc(protocol_size);
        memcpy(protocol, arguments[i] + 11, protocol_size);
        // printf("protocol = %s\n", protocol);
      } else if (strncmp(arguments[i] + 2, "ip_address=", 11) == 0) {
        ip_address_size = strlen(arguments[i] + 13) + 1;
        ip_address = malloc(ip_address_size);
        memcpy(ip_address, arguments[i] + 13, ip_address_size);
        // printf("ip_address = %s\n", ip_address);
      } else if (strncmp(arguments[i] + 2, "port=", 5) == 0) {
        port_size = strlen(arguments[i] + 7) + 1;
        port = malloc(port_size);
        memcpy(port, arguments[i] + 7, port_size);
        // printf("port = %s\n", port);
      } else if (strncmp(arguments[i] + 2, "robot_name=", 11) == 0) {
        robot_name_size = strlen(arguments[i] + 13) + 1;
        robot_name = malloc(robot_name_size);
        memcpy(robot_name, arguments[i] + 13, robot_name_size);
        // printf("robot_name = %s\n", robot_name);
      } else if (strncmp(arguments[i] + 2, "help", 4) == 0) {
        print_options();
        return false;
      } else {
        printf("Invalid option '%s'. Try '--help' for more information.\n", arguments[i]);
        return false;
      }
    } else {
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
  if (!protocol) {
    protocol = strdup("ipc");
    protocol_size = strlen(protocol);
  }

  // If no port is given, 1234 is used by default
  if (!port) {
    port = strdup("1234");
    port_size = strlen(port);
  }

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

  if (robot_name) {
    size_t with_robot_size = snprintf(NULL, 0, "%s%s%s", WEBOTS_CONTROLLER_URL, "/", robot_name) + 1;
    WEBOTS_CONTROLLER_URL = realloc(WEBOTS_CONTROLLER_URL, with_robot_size);
    sprintf(WEBOTS_CONTROLLER_URL, "%s%s%s", WEBOTS_CONTROLLER_URL, "/", robot_name);
  }
  putenv(WEBOTS_CONTROLLER_URL);

  // Show resulting target to user
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
    snprintf(NULL, 0, "%s%s%s%s%s%s%s%s", "Path=", WEBOTS_HOME, "\\lib\\controller:", WEBOTS_HOME,
             "\\msys64\\mingw64\\bin:", WEBOTS_HOME, "\\msys64\\mingw64\\bin\\cpp:", getenv("DYLD_LIBRARY_PATH")) +
    1;
  char *new_path = malloc(new_path_size);
  sprintf(new_path, "%s%s%s%s%s%s%s%s", "Path=", WEBOTS_HOME, "\\lib\\controller:", WEBOTS_HOME,
          "\\msys64\\mingw64\\bin:", WEBOTS_HOME, "\\msys64\\mingw64\\bin\\cpp:", getenv("DYLD_LIBRARY_PATH"));
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

  size_t webots_project_size = snprintf(NULL, 0, "%s%s", "WEBOTS_PROJECT=", project_path) + 1;
  char *webots_project = malloc(webots_project_size);
  sprintf(webots_project, "%s%s", "WEBOTS_PROJECT=", project_path);
  putenv(webots_project);

  // Add controller name to WEBOTS_CONTROLLER_NAME env variable
#ifdef _WIN32
  char *controller_name = strrchr(controller, '\\') + 1;
#else
  char *controller_name = strrchr(controller, '/') + 1;
#endif
  size_t webots_controller_name_size = snprintf(NULL, 0, "%s%s", "WEBOTS_CONTROLLER_NAME=", controller_name) + 1;
  char *webots_controller_name = malloc(webots_controller_name_size);
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
  char version[6];
  fscanf(version_file, "%[^\n]", version);
  fclose(version_file);

  size_t webots_version_size = snprintf(NULL, 0, "%s%s", "WEBOTS_VERSION=", version) + 1;
  char *webots_version = malloc(webots_version_size);
  sprintf(webots_version, "%s%s", "WEBOTS_VERSION=", version);
  putenv(webots_version);
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
  char *extension;
  if (!controller_name)
    extension = strrchr(controller, '.');
  else
    extension = strrchr(controller_name, '.');

  // Executable controller
  if (!extension) {
    exec_java_config_environment();
    system(controller);
  }
  // Python controller
  else if (strcmp(extension, ".py") == 0) {
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
  else if (strcmp(extension, ".m") == 0) {
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

    printf("%s\n", matlab_command);
    system(matlab_command);
  }
  // Java controller
  else if (strcmp(extension, ".jar") == 0 || strcmp(extension, ".class") == 0) {
    exec_java_config_environment();

    // Compute path to controller file
#ifdef _WIN32
    const size_t controller_file_size = strlen(strrchr(controller, '\\')) - 1;
#else
    const size_t controller_file_size = strlen(strrchr(controller, '/')) - 1;
#endif
    const size_t controller_size = strlen(controller);
    const size_t controller_path_size = controller_size - controller_file_size;
    char *controller_path = malloc(controller_path_size + 1);
    strncpy(controller_path, controller, controller_path_size);
    controller_path[controller_path_size] = '\0';

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
    controller_name[strlen(controller_name) - strlen(extension)] = '\0';
    size_t java_command_size = snprintf(NULL, 0, "%s%s%s%s%s", "java ", classpath, java_library, " ", controller_name + 1) + 1;
    char *java_command = malloc(java_command_size);
    sprintf(java_command, "%s%s%s%s%s", "java ", classpath, java_library, " ", controller_name + 1);

    // printf("%s\n", java_command);
    system(java_command);
  } else
    printf("The file extension '%s' is not supported as webots controller. Supported file types are executables, '.py', "
           "'.jar', '.class' and '.m'.\n",
           extension);

  return 0;
}
