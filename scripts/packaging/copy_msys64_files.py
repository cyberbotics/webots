#!/usr/bin/env python3

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

import subprocess
import os


def list_dependencies(package):
    return subprocess.check_output(['pactree', '-u', package]).decode().strip().split('\n')


# list all the pacman dependencies needed by Webots, including sub-dependencies
dependencies = list(set(  # use a set to make sure to avoid duplication
    list_dependencies('make') +
    list_dependencies('coreutils') +
    list_dependencies('mingw-w64-x86_64-gcc') +
    list_dependencies('mingw-w64-i686-gcc')
))

# add specific folder dependencies needed by Webots
folders = ['/tmp', '/mingw32', '/mingw32/bin', '/mingw32/lib', '/mingw64', '/mingw64/bin', '/mingw64/bin/cpp',
           '/mingw64/include',
           '/mingw64/include/libssh',
           '/mingw64/lib', '/mingw64/share',
           '/mingw64/share/qt6', '/mingw64/share/qt6/plugins', '/mingw64/share/qt6/translations',
           '/mingw64/share/qt6/plugins/imageformats', '/mingw64/share/qt6/plugins/platforms',
           '/mingw64/share/qt6/plugins/tls', '/mingw64/share/qt6/plugins/styles']
files = []
skip_paths = ['/usr/share/', '/mingw64/bin/zlib1.dll', '/mingw64/bin/libjpeg-8.dll']

# add all the files and folders corresponding to the pacman dependencies
for dependency in dependencies:
    print("# processing " + dependency, flush=True)
    for file in subprocess.check_output(['pacman', '-Qql', dependency]).decode().strip().split('\n'):
        skip = False
        for skip_path in skip_paths:
            if file.startswith(skip_path):
                skip = True
                break
        if skip:
            continue
        if not file.endswith('/'):
            files.append(file)
        else:
            folder = file.rstrip('/')
            if folder not in folders:
                folders.append(folder)

# write every dependency folder in the ISS file for folders
with open('msys64_folders.iss', 'w') as file:
    for folder in folders:
        file.write('Name: "{app}\\msys64' + folder.replace('/', '\\') + '"\n')

# add the dependencies provided in the files_msys64.txt file
root = subprocess.check_output(['cygpath', '-w', '/']).decode().strip().rstrip('\\')
with open('files_msys64.txt', 'r') as file:
    for line in file:
        line = line.strip()
        if not line.startswith('#') and line:
            if line in files:
                print('# \033[1;31m' + line + ' is already included\033[0m')
            else:
                files.append(line)

# automatically compute the dependencies of ffmpeg
print("# processing ffmpeg dependencies (DLLs)", flush=True)
for ffmpeg_dll in subprocess.check_output(['bash', 'ffmpeg_dependencies.sh'], shell=True).decode('utf-8').split():
    files.append('/mingw64/bin/' + ffmpeg_dll)

# write every dependency file in the ISS file for files
with open('msys64_files.iss', 'w') as iss_file:
    for file in files:
        file = file.replace('/', '\\')
        if file in ['\\mingw64\\bin\\libstdc++-6.dll',
                    '\\mingw64\\bin\\libgcc_s_seh-1.dll',
                    '\\mingw64\\bin\\libwinpthread-1.dll']:
            iss_file.write('Source: "' + root + file + '"; DestDir: "{app}\\msys64' + os.path.dirname(file) + '\\cpp"\n')
        else:
            iss_file.write('Source: "' + root + file + '"; DestDir: "{app}\\msys64' + os.path.dirname(file) + '"\n')
