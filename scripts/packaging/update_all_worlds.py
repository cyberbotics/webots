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

"""Load and save all the worlds in the 'projects' directory."""
import os
import fnmatch
import sys
from subprocess import call

if 'WEBOTS_HOME' in os.environ:
    WEBOTS_HOME = os.environ['WEBOTS_HOME']
else:
    raise RuntimeError('WEBOTS_HOME environmental variable is not set.')

worlds = []
for rootPath, dirNames, fileNames in os.walk(os.path.join(WEBOTS_HOME, 'projects')):
    for fileName in fnmatch.filter(fileNames, '*.wbt'):
        world = os.path.join(rootPath, fileName)
        worlds.append(world)

if sys.platform == 'win32':
    webotsFullPath = WEBOTS_HOME + os.sep + 'msys64' + os.sep + 'mingw64' + os.sep + 'bin' + os.sep + 'webots.exe'
else:
    webotsFullPath = os.environ['WEBOTS_HOME'] + os.sep + 'webots'

if not os.path.isfile(webotsFullPath):
    print(f'Error: webots binary not found at: {webotsFullPath}')
    sys.exit(1)

for i in range(len(worlds)):
    print('%d/%d: %s' % (i + 1, len(worlds), worlds[i]))
    call([webotsFullPath, worlds[i], '--minimize', '--batch', '--mode=pause', '--update-world'])
