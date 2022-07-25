%module vehicle

%pythonbegin %{
import sys
import os
if os.name == 'nt' and sys.version_info >= (3, 8):  # we need to explicitly list the folders containing the DLLs
    webots_home = os.path.normpath(os.environ['WEBOTS_HOME'])
    os.add_dll_directory(os.path.join(webots_home, 'lib', 'controller'))
    # MSYS2_HOME should be set by Webots or ~/.bash_profile
    # if not set, we are in the case of an extern controller and a regularly installed version of Webots
    msys64_root = os.environ['MSYS2_HOME'] if 'MSYS2_HOME' in os.environ else os.path.join(webots_home, 'msys64')
    cpp_folder = os.path.join(msys64_root, 'mingw64', 'bin', 'cpp')
    if not os.path.isdir(cpp_folder):  # development environment
        cpp_folder = os.path.join(msys64_root, 'mingw64', 'bin')
    os.add_dll_directory(cpp_folder)
%}

%{
#include <webots/vehicle/Driver.hpp>
#include <webots/vehicle/Car.hpp>
%}

%import "controller.i"

%include <webots/vehicle/Driver.hpp>
%include <webots/vehicle/Car.hpp>
