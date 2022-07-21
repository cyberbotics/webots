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


worlds = []
root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
for rootPath, dirNames, fileNames in os.walk(os.path.join(root, 'projects')):
    for fileName in fnmatch.filter(fileNames, '*.wbt'):
        world = os.path.join(rootPath, fileName)
        worlds.append(world)
webotsFullPath = None
if sys.platform == 'win32':
    webotsFullPath = os.path.join(os.environ['WEBOTS_HOME'], 'msys64', 'mingw64', 'bin', 'webots.exe')
else:
    webotsBinary = 'webots'
    if 'WEBOTS_HOME' in os.environ:
        webotsFullPath = os.path.join(os.environ['WEBOTS_HOME'], webotsBinary)
    else:
        webotsFullPath = os.path.join('..', '..', webotsBinary)
    if not os.path.isfile(webotsFullPath):
        print('Error: ' + webotsBinary + ' binary not found')
        sys.exit(1)
    webotsFullPath = os.path.normpath(webotsFullPath)

for i in range(len(worlds)):
    print('%d/%d: %s' % (i + 1, len(worlds), worlds[i]))
    call([webotsFullPath, worlds[i], '--minimize', '--batch', '--mode=pause', '--update-world'])
