# Copyright 1996-2023 Cyberbotics Ltd.
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
elif sys.platform == 'darwin':
    path = os.path.join('Contents', 'lib', 'controller', 'libController.dylib')

wb = ctypes.cdll.LoadLibrary(os.path.join(os.environ['WEBOTS_HOME'], path))

if sys.platform == 'win32':
    ctypes.cdll.LoadLibrary(os.path.join(os.environ['WEBOTS_HOME'], 'lib', 'controller', 'generic_robot_window.dll'))
