# Copyright 1996-2024 Cyberbotics Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import ctypes
import os
import sys

if sys.platform == 'linux' or sys.platform == 'linux2':
    path = os.path.join('lib', 'controller', 'libController.so')
elif sys.platform == 'win32':
    path = os.path.join('lib', 'controller', 'Controller.dll')
    # Since Python 3.8, dependent DLLs are no longer found via PATH on Windows.
    # Add directories containing required runtime DLLs to the search path.
    webots_home = os.environ['WEBOTS_HOME']
    for dll_dir in [
        os.path.join(webots_home, 'lib', 'controller'),
        # In the installed distribution, MinGW runtime DLLs (libgcc_s_seh-1.dll,
        # libwinpthread-1.dll, libstdc++-6.dll) are placed in msys64/mingw64/bin/cpp.
        os.path.join(webots_home, 'msys64', 'mingw64', 'bin', 'cpp'),
        os.path.join(webots_home, 'msys64', 'mingw64', 'bin'),
    ]:
        if os.path.isdir(dll_dir):
            os.add_dll_directory(dll_dir)
    # For development builds running from source, also check the system MSYS2 installation.
    mingw_bin = os.path.join(os.environ.get('MSYS2_HOME', os.path.join(os.path.splitdrive(sys.executable)[0] + os.sep,
                             'msys64')), 'mingw64', 'bin')
    if os.path.isdir(mingw_bin):
       os.add_dll_directory(mingw_bin)
elif sys.platform == 'darwin':
    path = os.path.join('Contents', 'lib', 'controller', 'libController.dylib')

wb = ctypes.cdll.LoadLibrary(os.path.join(os.environ['WEBOTS_HOME'], path))

if sys.platform == 'win32':
    ctypes.cdll.LoadLibrary(os.path.join(os.environ['WEBOTS_HOME'], 'lib', 'controller', 'generic_robot_window.dll'))
