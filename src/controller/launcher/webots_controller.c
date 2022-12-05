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

bool get_webots_home() {
  if (!getenv("WEBOTS_HOME")) {
    printf("Set the path to your webots installation folder in WEBOTS_HOME environment variable.\n");
    return false;
  } else
    WEBOTS_HOME = malloc(strlen(getenv("WEBOTS_HOME")) + 1);
  strcpy(WEBOTS_HOME, getenv("WEBOTS_HOME"));
  return true;
}

void print_options() {
  printf(
    "Usage: webots_controller [options] [controller_file]\n\nOptions:\n\n  --protocol=<ipc|tcp>\n    IPC is used by "
    "default. IPC should be used when Webots is running on the same machine as the extern controller. TCP should be used when "
    "connecting to a remote instance of Webots.\n\n  --ip_address=<ip_address>\n    The IP address of the remote machine on "
    "which the Webots instance is running. This option should only be used with the TCP protocol (remote controllers).\n\n  "
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

  // Write WEBOTS_CONTROLLER_URL in function of given options
  if (strncmp(protocol, "tcp", 3) == 0) {
    if (!ip_address) {
      printf("Specify the IP address of the Webots machine to connect to with '--ip_address=' option.\n");
      return false;
    }
    char *WEBOTS_CONTROLLER_URL = malloc(protocol_size + ip_address_size + port_size + robot_name_size + 28);
    memcpy(WEBOTS_CONTROLLER_URL, "WEBOTS_CONTROLLER_URL=", 22);
    strcat(WEBOTS_CONTROLLER_URL, protocol);
    strcat(WEBOTS_CONTROLLER_URL, "://");
    strcat(WEBOTS_CONTROLLER_URL, ip_address);
    strcat(WEBOTS_CONTROLLER_URL, ":");
    strcat(WEBOTS_CONTROLLER_URL, port);
    if (robot_name) {
      strcat(WEBOTS_CONTROLLER_URL, "/");
      strcat(WEBOTS_CONTROLLER_URL, robot_name);
    }
    putenv(WEBOTS_CONTROLLER_URL);
  } else if (strncmp(protocol, "ipc", 3) == 0) {
    if (ip_address)
      printf("Ignoring IP address for ipc protocol.\n");
    char *WEBOTS_CONTROLLER_URL = malloc(protocol_size + port_size + robot_name_size + 27);
    memcpy(WEBOTS_CONTROLLER_URL, "WEBOTS_CONTROLLER_URL=", 22);
    strcat(WEBOTS_CONTROLLER_URL, protocol);
    strcat(WEBOTS_CONTROLLER_URL, "://");
    strcat(WEBOTS_CONTROLLER_URL, port);
    if (robot_name) {
      strcat(WEBOTS_CONTROLLER_URL, "/");
      strcat(WEBOTS_CONTROLLER_URL, robot_name);
    }
    putenv(WEBOTS_CONTROLLER_URL);
  } else {
    printf("Only ipc and tcp protocols are supported.\n");
    return false;
  }

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
  char *lib_controller = malloc(3 * strlen(WEBOTS_HOME) + 61);
  memcpy(lib_controller, WEBOTS_HOME, strlen(WEBOTS_HOME));
  strcat(lib_controller, "\\lib\\controller:");
  char *bin = malloc(strlen(WEBOTS_HOME) + 21);
  memcpy(bin, WEBOTS_HOME, strlen(WEBOTS_HOME));
  strcat(bin, "\\msys64\\mingw64\\bin:");
  strcat(lib_controller, bin);
  char *cpp = malloc(strlen(WEBOTS_HOME) + 25);
  memcpy(cpp, WEBOTS_HOME, strlen(WEBOTS_HOME));
  strcat(cpp, "\\msys64\\mingw64\\bin\\cpp:");
  strcat(lib_controller, cpp);
  const size_t Path_size = getenv("Path") ? strlen(getenv("Path")) : 0;
  char *new_path = malloc(strlen(lib_controller) + Path_size + 6);
  memcpy(new_path, "Path=", 5);
  strcat(new_path, lib_controller);
  if (getenv("Path"))
    strcat(new_path, getenv("Path"));
  putenv(new_path);
#elif defined __linux__
  char *lib_controller = malloc(strlen(WEBOTS_HOME) + 17);
  memcpy(lib_controller, WEBOTS_HOME, strlen(WEBOTS_HOME));
  strcat(lib_controller, "/lib/controller:");
  const size_t LD_LIBRARY_PATH_size = getenv("LD_LIBRARY_PATH") ? strlen(getenv("LD_LIBRARY_PATH")) : 0;
  char *new_ld_path = malloc(strlen(lib_controller) + LD_LIBRARY_PATH_size + 17);
  memcpy(new_ld_path, "LD_LIBRARY_PATH=", 16);
  strcat(new_ld_path, lib_controller);
  if (getenv("LD_LIBRARY_PATH"))
    strcat(new_ld_path, getenv("LD_LIBRARY_PATH"));
  putenv(new_ld_path);
#elif defined __APPLE__
  char *lib_controller = malloc(strlen(WEBOTS_HOME) + 26);
  memcpy(lib_controller, WEBOTS_HOME, strlen(WEBOTS_HOME));
  strcat(lib_controller, "/Contents/lib/controller:");
  const size_t DYLD_LIBRARY_PATH_size = getenv("DYLD_LIBRARY_PATH") ? strlen(getenv("DYLD_LIBRARY_PATH")) : 0;
  char *new_ld_path = malloc(strlen(lib_controller) + DYLD_LIBRARY_PATH_size + 19);
  memcpy(new_ld_path, "DYLD_LIBRARY_PATH=", 18);
  strcat(new_ld_path, lib_controller);
  if (getenv("DYLD_LIBRARY_PATH"))
    strcat(new_ld_path, getenv("DYLD_LIBRARY_PATH"));
  putenv(new_ld_path);
#endif
}

void python_config_environment() {
#ifdef _WIN32
  char *lib_controller = malloc(strlen(WEBOTS_HOME) + 24);
  memcpy(lib_controller, WEBOTS_HOME, strlen(WEBOTS_HOME));
  strcat(lib_controller, "\\lib\\controller\\python:");
  const size_t PYTHONPATH_size = getenv("PYTHONPATH") ? strlen(getenv("PYTHONPATH")) : 0;
  char *new_python_path = malloc(strlen(lib_controller) + PYTHONPATH_size + 12);
  memcpy(new_python_path, "PYTHONPATH=", 11);
  strcat(new_python_path, lib_controller);
  if (getenv("PYTHONPATH"))
    strcat(new_python_path, getenv("PYTHONPATH"));
  putenv(new_python_path);
  char python_ioencoding[22] = "PYTHONIOENCODING=UTF-8";
  putenv(python_ioencoding);

  // CHECK FOR EPUCK ?
#elif defined __linux__
  char *lib_controller = malloc(strlen(WEBOTS_HOME) + 24);
  memcpy(lib_controller, WEBOTS_HOME, strlen(WEBOTS_HOME));
  strcat(lib_controller, "/lib/controller/python:");
  const size_t PYTHONPATH_size = getenv("PYTHONPATH") ? strlen(getenv("PYTHONPATH")) : 0;
  char *new_python_path = malloc(strlen(lib_controller) + PYTHONPATH_size + 12);
  memcpy(new_python_path, "PYTHONPATH=", 11);
  strcat(new_python_path, lib_controller);
  if (getenv("PYTHONPATH"))
    strcat(new_python_path, getenv("PYTHONPATH"));
  putenv(new_python_path);
  char python_ioencoding[22] = "PYTHONIOENCODING=UTF-8";
  putenv(python_ioencoding);
#elif defined __APPLE__
  char *lib_controller = malloc(strlen(WEBOTS_HOME) + 24);
  memcpy(lib_controller, WEBOTS_HOME, strlen(WEBOTS_HOME));
  strcat(lib_controller, "/Contents/lib/controller/python:");
  const size_t PYTHONPATH_size = getenv("PYTHONPATH") ? strlen(getenv("PYTHONPATH")) : 0;
  char *new_python_path = malloc(strlen(lib_controller) + PYTHONPATH_size + 12);
  memcpy(new_python_path, "PYTHONPATH=", 11);
  strcat(new_python_path, lib_controller);
  if (getenv("PYTHONPATH"))
    strcat(new_python_path, getenv("PYTHONPATH"));
  putenv(new_python_path);
  char python_ioencoding[22] = "PYTHONIOENCODING=UTF-8";
  putenv(python_ioencoding);
#endif
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
    char *python_command = malloc(strlen(controller) + 9);
#ifdef _WIN32
    memcpy(python_command, "python ", 7);
#else
    memcpy(python_command, "python3 ", 8);
#endif
    strcat(python_command, controller);
    system(python_command);
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
    memcpy(controller_path, controller, controller_path_size);
    controller_path[controller_path_size] = '\0';

    // Write path to java lib controller
    char *lib_controller = malloc(strlen(WEBOTS_HOME) + 30);
    memcpy(lib_controller, WEBOTS_HOME, strlen(WEBOTS_HOME));
#ifdef _WIN32
    strcat(lib_controller, "\\lib\\controller\\java");
#elif defined __APPLE__
    strcat(lib_controller, "/Contents/lib/controller/java");
#elif defined __linux__
    strcat(lib_controller, "/lib/controller/java");
#endif

    // Write the 'classpath' option (mandatory for java controllers)
    char *classpath = malloc(strlen(lib_controller) + controller_path_size + 28);
    memcpy(classpath, "-classpath ", 11);
    strcat(classpath, lib_controller);
#ifdef _WIN32
    strcat(classpath, "\\Controller.jar:");
#elif defined __APPLE__
    strcat(classpath, "/Controller.jar:");
#elif defined __linux__
    strcat(classpath, "/Controller.jar:");
#endif
    strcat(classpath, controller_path);

    // Write the 'Djava.library.path' option (mandatory for java controllers)
    char *java_library = malloc(strlen(lib_controller) + 22);
    memcpy(java_library, " -Djava.library.path=", 21);
    strcat(java_library, lib_controller);

    // Write arguments to java command
    char *java_command = malloc(strlen(classpath) + strlen(java_library) + strlen(controller_name) + 8);
    memcpy(java_command, "java ", 5);
    strcat(java_command, classpath);
    strcat(java_command, java_library);
    strcat(java_command, " ");
    controller_name[strlen(controller_name) - strlen(extension)] = '\0';
    strcat(java_command, controller_name + 1);
    // printf("%s\n", java_command);
    system(java_command);
  } else
    printf("The file extension '%s' is not supported as webots controller. Supported file types are executables, '.py', "
           "'.jar', '.class' and '.m'.\n",
           extension);

  return 0;
}