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

"""Update rpaths."""

import commands
import os
import re
import sys


WEBOTS_HOME = os.getenv('WEBOTS_HOME')


def command(cmd):
    result = commands.getstatusoutput(cmd)
    if result[0] != 0:
        raise RuntimeError('cmd failed: "' + cmd + '"')
    return result[1]


os.chdir(WEBOTS_HOME)
dylibFiles = command('find . -type f -name *dylib -o -name *jnilib | grep -v dependencies | grep -v ros | grep -v sumo | '
                     'grep -v nao_soccer | grep -v dashel | grep -v build').split()
frameworkFiles = command('find Contents -name *.framework | '
                         'sed -e "s:\\(.*\\)/\\([^/]*\\).framework:\\1/\\2.framework/\\2:"').split()
for i in range(len(frameworkFiles)):
    frameworkFiles[i] = os.path.realpath(frameworkFiles[i]).replace(WEBOTS_HOME, '')
    frameworkFiles[i] = re.sub(r"^\.", "", frameworkFiles[i])
    frameworkFiles[i] = re.sub(r"^/", "", frameworkFiles[i])
controllerFiles = command('find projects resources -name controllers | xargs -I{} find {} -maxdepth 1 -mindepth 1 -type d | '
                          'grep -v ros | grep -v thymio2_aseba | '
                          'sed -e "s:\\(.*\\)/\\([^/]*\\):\\1/\\2/\\2:" | '
                          'perl -ne \'chomp(); if (-e $_) {print "$_\n"}\' ').split()
binaryFiles = [
    'Contents/MacOS/webots'
]
qtBinaryFiles = [
    'bin/qt/moc',
    'bin/qt/lrelease',
    'bin/qt/lupdate'
]

success = True

# Check dependencies are:
# - absolute (system) and are not containing local (macports)
# - relative to @rpath (= WEBOTS_HOME) and are existing
for f in dylibFiles + frameworkFiles + controllerFiles + binaryFiles + qtBinaryFiles:
    dependencies = command('otool -L ' + f + ' | grep -v ' + f + ': | sed -e "s: (compatibility.*::" | '
                           'sed -e "s:^[ \t]*::"').split('\n')
    for d in dependencies:
        if (not d.startswith('/') and not d.startswith('@rpath/')) or 'local' in d:
            success = False
            sys.stderr.write('Dependency error:\n')
            sys.stderr.write('- File: ' + f + '\n')
            sys.stderr.write('- Dependency: ' + d + '\n')
        elif d.startswith('@rpath/'):
            expectedFile = d.replace('@rpath', WEBOTS_HOME)
            if not os.path.exists(expectedFile):
                success = False
                sys.stderr.write('Dependency error:\n')
                sys.stderr.write('- File: ' + f + '\n')
                sys.stderr.write('- Dependency: ' + d + '\n')

# check that the binaries have an RPATH
for f in controllerFiles + binaryFiles + qtBinaryFiles:
    rpath = command('otool -l ' + f + ' | grep LC_RPATH -A 3 | grep path | cut -c15- | cut -d\' \' -f1')
    if rpath is None or not rpath:
        success = False
        sys.stderr.write('RPATH not defined in:\n')
        sys.stderr.write('- File: ' + f + '\n')
    elif not rpath.startswith('@loader_path/') and not rpath.startswith('@rpath/') and f not in qtBinaryFiles:
        success = False
        sys.stderr.write('RPATH error in:\n')
        sys.stderr.write('- File: ' + f + '\n')
        sys.stderr.write('- rpath: ' + rpath + '\n')


if success:
    print('Dylibs dependencies: ok')
    print('Frameworks dependencies: ok')
else:
    sys.exit(1)  # Quit the script with an error code.
