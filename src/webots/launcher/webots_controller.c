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
        protocol = malloc(strlen(arguments[i] + 11) + 1);
        strcpy(protocol, arguments[i] + 11);
        protocol_size = strlen(protocol);
        // printf("protocol = %s\n", protocol);
      } else if (strncmp(arguments[i] + 2, "ip_address=", 11) == 0) {
        ip_address = malloc(strlen(arguments[i] + 13) + 1);
        strcpy(ip_address, arguments[i] + 13);
        ip_address_size = strlen(ip_address);
        // printf("ip_address = %s\n", ip_address);
      } else if (strncmp(arguments[i] + 2, "port=", 5) == 0) {
        port = malloc(strlen(arguments[i] + 7) + 1);
        strcpy(port, arguments[i] + 7);
        port_size = strlen(port);
        // printf("port = %s\n", port);
      } else if (strncmp(arguments[i] + 2, "robot_name=", 11) == 0) {
        robot_name = malloc(strlen(arguments[i] + 13) + 1);
        strcpy(robot_name, arguments[i] + 13);
        robot_name_size = strlen(robot_name);
        // printf("robot_name = %s\n", robot_name);
      } else if (strncmp(arguments[i] + 2, "help", 4) == 0) {
        print_options();
        return false;
      } else {
        printf("Invalid option '%s'. Try '--help' for more information.\n", arguments[i]);
        return false;
      }
    } else {
      controller = malloc(strlen(arguments[i]));
      strcpy(controller, arguments[i]);
    }
  }

  // Check that a controller path has been provided
  if (!controller) {
    printf("No controller file provided. Please provide an existing controller file as argument.\n");
    return false;
  }

  // If no protocol is given, ipc is used by default
  if (!protocol) {
    printf("Using default ipc protocol.\n");
    protocol = malloc(4);
    strcpy(protocol, "ipc");
    protocol_size = strlen(protocol);
  }

  // If no port is given, 1234 is used by default
  if (!port) {
    printf("Using default port 1234.\n");
    port = malloc(5);
    strcpy(port, "1234");
    port_size = strlen(port);
  }

  // Write WEBOTS_CONTROLLER_URL in function of given options
  if (strncmp(protocol, "tcp", 3) == 0) {
    if (!ip_address) {
      printf("Specify the IP address of the Webots machine to connect to with '--ip_address=' option.\n");
      return false;
    }
    char *WEBOTS_CONTROLLER_URL = malloc(protocol_size + ip_address_size + port_size + robot_name_size + 28);
    strcat(WEBOTS_CONTROLLER_URL, "WEBOTS_CONTROLLER_URL=");
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
    strcat(WEBOTS_CONTROLLER_URL, "WEBOTS_CONTROLLER_URL=");
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

  // printf("%s\n", getenv("WEBOTS_CONTROLLER_URL"));
  return true;
}

void exec_java_config_environment() {
#ifdef _WIN32
  char lib_controller[2048] = "";
  strcat(lib_controller, WEBOTS_HOME);
  strcat(lib_controller, "\\lib\\controller:");
  char bin[256] = "";
  strcat(bin, WEBOTS_HOME);
  strcat(bin, "\\msys64\\mingw64\\bin:");
  strcat(lib_controller, bin);
  char cpp[256] = "";
  strcat(cpp, WEBOTS_HOME);
  strcat(cpp, "\\msys64\\mingw64\\bin\\cpp:");
  strcat(lib_controller, cpp);
  char new_path[4096] = "Path=";
  strcat(new_path, lib_controller);
  if (getenv("Path"))
    strcat(new_path, getenv("Path"));
  putenv(new_path);
#elif defined __linux__
  char lib_controller[256] = "";
  strcat(lib_controller, WEBOTS_HOME);
  strcat(lib_controller, "/lib/controller:");
  char new_ld_path[4096] = "LD_LIBRARY_PATH=";
  strcat(new_ld_path, lib_controller);
  if (getenv("LD_LIBRARY_PATH"))
    strcat(new_ld_path, getenv("LD_LIBRARY_PATH"));
  putenv(new_ld_path);
#elif defined __APPLE__
  char lib_controller[256] = "";
  strcat(lib_controller, WEBOTS_HOME);
  strcat(lib_controller, "/Contents/lib/controller:");
  char new_ld_path[4096] = "DYLD_LIBRARY_PATH=";
  strcat(new_ld_path, lib_controller);
  if (getenv("DYLD_LIBRARY_PATH"))
    strcat(new_ld_path, getenv("DYLD_LIBRARY_PATH"));
  putenv(new_ld_path);
#endif
}

void python_config_environment() {
#ifdef _WIN32
  char lib_controller[256] = "";
  strcat(lib_controller, WEBOTS_HOME);
  strcat(lib_controller, "\\lib\\controller\\python:");
  char new_python_path[4096] = "PYTHONPATH=";
  strcat(new_python_path, lib_controller);
  if (getenv("PYTHONPATH"))
    strcat(new_python_path, getenv("PYTHONPATH"));
  putenv(new_python_path);
  putenv("PYTHONIOENCODING=UTF-8");

  // CHECK FOR EPUCK ?
#elif defined __linux__
  char lib_controller[256] = "";
  strcat(lib_controller, WEBOTS_HOME);
  strcat(lib_controller, "/lib/controller/python:");
  char new_python_path[4096] = "PYTHONPATH=";
  strcat(new_python_path, lib_controller);
  if (getenv("PYTHONPATH"))
    strcat(new_python_path, getenv("PYTHONPATH"));
  putenv(new_python_path);
  putenv("PYTHONIOENCODING=UTF-8");
#elif defined __APPLE__
  char lib_controller[256] = "";
  strcat(lib_controller, WEBOTS_HOME);
  strcat(lib_controller, "/Contents/lib/controller/python:");
  char new_python_path[4096] = "PYTHONPATH=";
  strcat(new_python_path, lib_controller);
  if (getenv("PYTHONPATH"))
    strcat(new_python_path, getenv("PYTHONPATH"));
  putenv(new_python_path);
  putenv("PYTHONIOENCODING=UTF-8");
#endif
}

int main(int argc, char **argv) {
  // Check WEBOTS_HOME and exit if empty
  bool is_set = get_webots_home();
  if (!is_set)
    return -1;

  bool success = parse_options(argc, argv);
  if (!success)
    return -1;

  // Get controller file and check if exists
  if (access(controller, F_OK) != 0) {
    printf("Controller file '%s' not found. Please specify a path to an existing controller file.\n", controller);
    return -1;
  }

  // Get extension from controller name (relative paths robustness)
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
    // printf("C or C++ file\n");
    exec_java_config_environment();
    int status = system(controller);
  }
  // Python controller
  else if (strcmp(extension, ".py") == 0) {
    // printf("Python file\n");
    python_config_environment();
#ifdef _WIN32
    char python_command[256] = "python ";
#else
    char python_command[256] = "python3 ";
#endif
    strcat(python_command, controller);
    int status = system(python_command);
  }
  // Java controller
  else if (strcmp(extension, ".jar") == 0 || strcmp(extension, ".class") == 0) {
    // printf("Java file\n");
    exec_java_config_environment();
    char lib_controller[256] = "";
    char classpath[512] = "-classpath ";
    char java_library[256] = " -Djava.library.path=";
    strcat(lib_controller, WEBOTS_HOME);
#ifdef _WIN32
    strcat(lib_controller, "\\lib\\controller\\java");
    strcat(classpath, lib_controller);
    strcat(classpath, "\\Controller.jar:");
    size_t controller_file_size = strlen(strrchr(controller, '\\')) - 1;
#elif defined __APPLE__
    strcat(lib_controller, "/Contents/lib/controller/java");
    strcat(classpath, lib_controller);
    strcat(classpath, "/Controller.jar:");
    size_t controller_file_size = strlen(strrchr(controller, '/')) - 1;
#elif defined __linux__
    strcat(lib_controller, "/lib/controller/java");
    strcat(classpath, lib_controller);
    strcat(classpath, "/Controller.jar:");
    size_t controller_file_size = strlen(strrchr(controller, '/')) - 1;
#endif
    size_t controller_size = strlen(controller);
    size_t controller_path_size = controller_size - controller_file_size;
    char controller_path[controller_path_size + 1];
    strncpy(controller_path, controller, controller_path_size);
    controller_path[controller_path_size] = '\0';
    strcat(classpath, controller_path);
    strcat(java_library, lib_controller);

    // Write arguments to java command
    char java_command[1024] = "java ";
    strcat(java_command, classpath);
    strcat(java_command, java_library);
    strcat(java_command, " ");
    const size_t extension_size = strlen(extension);
    controller_name[strlen(controller_name) - extension_size] = '\0';
    strcat(java_command, controller_name + 1);
    int status = system(java_command);
  } else
    printf("This file type is not supported as webots controller.\n");

  return 0;
}