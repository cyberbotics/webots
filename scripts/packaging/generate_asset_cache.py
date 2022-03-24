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

"""Generate and package the asset cache"""
import os
from pathlib import Path
import re
import sys
import shutil
import hashlib

EXTENSIONS = [
    'jpg', 'JPG', 'png', 'PNG', 'jpeg', 'JPEG', 'stl', 'STL', 'dae', 'DAE', 'obj', 'OBJ', 'mp3', 'MP3', 'wav', 'WAV'
]

# ensure WEBOTS_HOME is set and tag was provided
if 'WEBOTS_HOME' in os.environ:
    WEBOTS_HOME = os.environ['WEBOTS_HOME']
else:
    raise RuntimeError('Error, WEBOTS_HOME variable is not set.')

if len(sys.argv) != 2:
    sys.exit('Missing argument: commit sha or tag.')
else:
    tag = sys.argv[1]

with open(os.path.join(WEBOTS_HOME, 'resources', 'version.txt'), 'r') as file:
    folder_name = f'assets-{file.readline().strip()}'

# retrieve all assets
assets = []
for extension in EXTENSIONS:
    assets.extend(Path(WEBOTS_HOME + '/projects').rglob(f'*.{extension}'))
    assets.extend(Path(WEBOTS_HOME + '/tests').rglob(f'*.{extension}'))

# create and fill asset folder
if os.path.exists(folder_name):
    shutil.rmtree(folder_name)

os.mkdir(folder_name)
for asset in assets:
    # generate hash of the remote url
    remote_url = str(asset).replace(WEBOTS_HOME, rf'https://raw.githubusercontent.com/cyberbotics/webots/{tag}')
    hash = hashlib.sha1(remote_url.encode('utf-8')).hexdigest()
    # copy to asset folder
    shutil.copyfile(asset, f'./{folder_name}/{hash}')

# generate zip file
shutil.make_archive(folder_name, 'zip', folder_name)
shutil.move(f'{folder_name}.zip', f'{WEBOTS_HOME}/distribution/{folder_name}.zip')
shutil.rmtree(folder_name)
