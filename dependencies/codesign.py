#!/usr/bin/env python

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

"""Fix the @rpath in the Qt libraries on macOS."""

import glob
import os

WEBOTS_HOME = os.environ['WEBOTS_HOME']
framework_files = glob.glob(WEBOTS_HOME + '/Contents/Frameworks/*.framework')
plugin_files = glob.glob(WEBOTS_HOME + '/Contents/lib/webots/qt/plugins/*/*.dylib')
for file in framework_files + plugin_files:
    os.system(f'codesign --force -s - {file}')
