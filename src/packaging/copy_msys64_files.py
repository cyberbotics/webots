#!/usr/bin/env python

# Copyright 1996-2018 Cyberbotics Ltd.
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

import subprocess
import os
import sys


def list_dependencies(package):
    return subprocess.check_output(['pactree', '-u', package]).decode().strip().split('\n')


d = list_dependencies('make')
d += list_dependencies('coreutils')
d += list_dependencies('gcc')
d += list_dependencies('mingw-w64-i686-gcc')

# remove duplicate packages
seen = set()
dependencies = []
for item in d:
    if item not in seen:
        seen.add(item)
        dependencies.append(item)

folders = ['/tmp', '/mingw32', '/mingw32/bin', '/mingw32/lib', '/mingw64', '/mingw64/bin', '/mingw64/include',
           '/mingw64/bin/platforms/',  # hack to get qwindows.dll found by Webots
           '/mingw64/include/libssh', '/mingw64/lib', '/mingw64/share',
           '/mingw64/share/qt5', '/mingw64/share/qt5/plugins', '/mingw64/share/qt5/translations',
           '/mingw64/share/qt5/plugins/imageformats', '/mingw64/share/qt5/plugins/platforms',
           '/mingw64/share/qt5/plugins/printsupport', '/mingw64/share/qt5/plugins/styles']
files = []
skip_paths = ['/usr/share/', '/mingw64/bin/zlib1.dll']
for p in dependencies:
    print("# processing " + p)
    sys.stdout.flush()
    l = subprocess.check_output(['pacman', '-Qql', p]).decode().strip().split('\n')
    for f in l:
        skip = False
        for g in skip_paths:
            if f.startswith(g):
                skip = True
                break
        if skip:
            continue
        if not f.endswith('/'):
            files.append(f)
        else:
            g = f[:-1]
            if g not in folders:
                folders.append(g)

f = open('msys64_folders.iss', 'w')
for i in folders:
    f.write('Name: "{app}\\msys64' + i.replace('/', '\\') + '"\n')
f.close()

root = subprocess.check_output(['cygpath', '-w', '/']).decode().strip()[:-1]
with open('files_msys64.txt', 'r') as f:
    for line in f:
        l = line.strip()
        if not l.startswith('#') and l:
            if l in files:
                print('# \033[1;31m' + l + ' is already included\033[0m')
            else:
                files.append(l)
f = open('msys64_files.iss', 'w')
for i in files:
    w = i.replace('/', '\\')
    f.write('Source: "' + root + w + '"; DestDir: "{app}\\msys64' + os.path.dirname(w) + '"\n')
# This is a patch needed to ensure qwindows.dll is found by Webots (it should be improved)
f.write('Source: "' + root + '\\mingw64\\share\\qt5\\plugins\\platforms\\qwindows.dll"; DestDir: ' +
        '"{app}\\msys64\\mingw64\\bin\\platforms"\n')
f.close()
