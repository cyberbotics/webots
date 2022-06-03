#!/usr/bin/env python

# Copyright 1996-2022 Cyberbotics Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Fix the @rpath in the Qt libraries on macOS."""

import glob
import os

WEBOTS_HOME = os.environ['WEBOTS_HOME']
framework_files = glob.glob(WEBOTS_HOME + '/Frameworks/*.framework')
frameworks = []
for framework in framework_files:
    frameworks.append(framework[framework.rfind('/')+1:-10])

for framework in frameworks:
    file = f'{WEBOTS_HOME}/Frameworks/{framework}.framework/Versions/A/{framework}'
    os.system(f'install_name_tool -id @rpath/Frameworks/{framework}.framework/Versions/A/{framework} {file}')
    for f in frameworks:
        os.system(f'install_name_tool -change @rpath/Contents/Frameworks/{f}.framework/Versions/A/{f} @rpath/Frameworks/{f}.framework/Versions/A/{f} {file}')
    os.system(f'codesign --verbose=0 --force -s - {file}')
