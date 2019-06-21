#!/usr/bin/env python

# Copyright 1996-2019 Cyberbotics Ltd.
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
for rootPath, dirNames, fileNames in os.walk(os.environ['WEBOTS_HOME'] + os.sep + 'projects'):
    for fileName in fnmatch.filter(fileNames, '*.wbt'):
        world = os.path.join(rootPath, fileName)
        worlds.append(world)
webotsFullPath = None
if sys.platform == 'win32':
    webotsFullPath = os.environ['WEBOTS_HOME'] + os.sep + 'msys64' + os.sep + 'mingw64' + os.sep + 'bin' + os.sep + 'webots.exe'
else:
    webotsBinary = 'webots'
    if 'WEBOTS_HOME' in os.environ:
        webotsFullPath = os.environ['WEBOTS_HOME'] + os.sep + webotsBinary
    else:
        webotsFullPath = '..' + os.sep + '..' + os.sep + webotsBinary
    if not os.path.isfile(webotsFullPath):
        print('Error: ' + webotsBinary + ' binary not found')
        sys.exit(1)
    webotsFullPath = os.path.normpath(webotsFullPath)

for i in range(len(worlds)):
    sys.stdout.write("\r%d/%d" % (i + 1, len(worlds)))
    sys.stdout.flush()
    call([webotsFullPath, worlds[i], "--minimize", "--mode=pause", "--update-world"])
sys.stdout.write("\n")
