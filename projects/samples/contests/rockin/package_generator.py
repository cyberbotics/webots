#!/usr/bin/env python

# Copyright 1996-2021 Cyberbotics Ltd.
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

import os
import glob
import zipfile

# simple files to zip
files_to_zip = ['version.txt', 'change_log.txt'] + \
    glob.glob('worlds' + os.sep + '*.wbt') + \
    glob.glob('worlds' + os.sep + '.*.wbproj') + \
    glob.glob('worlds' + os.sep + 'textures' + os.sep + '*.png') + \
    glob.glob('protos' + os.sep + '*.proto') + \
    glob.glob('protos' + os.sep + 'textures' + os.sep + '*.png')

# get version
version = ''
with open("version.txt", "r") as file:
    version = file.readlines()[0].strip()

zip_file = 'rockin_v' + version + '.zip'

# create the archive
print('creating archive ' + zip_file)
with zipfile.ZipFile(zip_file, 'w') as zip_file:
    for file in files_to_zip:
        print('adding ' + file)
        zip_file.write(file)
print('closing')
zip_file.close()
