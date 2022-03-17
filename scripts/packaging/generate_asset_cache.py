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

# ensure WEBOTS_HOME is set and tag was provided
if 'WEBOTS_HOME' in os.environ:
    WEBOTS_HOME = os.environ['WEBOTS_HOME']
else:
    raise RuntimeError('Error, WEBOTS_HOME variable is not set.')

if len(sys.argv) != 2:
    sys.exit('Missing argument: commit sha or tag.')
else:
    tag = sys.argv[1]

# retrieve all files that potentially contain assets
paths = []
paths.extend(Path(WEBOTS_HOME + '/projects').rglob('*.proto'))
paths.extend(Path(WEBOTS_HOME + '/projects').rglob('*.wbt'))
paths.extend(Path(WEBOTS_HOME + '/tests').rglob('*.wbt'))
paths.extend(Path(WEBOTS_HOME + '/resources/nodes').rglob('*.wrl'))

with open(WEBOTS_HOME + '/scripts/packaging/controllers_with_urls.txt', 'r') as files:
    paths.extend(list(map(lambda path: WEBOTS_HOME + path, files.read().splitlines())))

# retrieve the urls of the assets themselves
base_url = f'https://raw.githubusercontent.com/cyberbotics/webots/{tag}/'
possible_extensions = 'jpg|JPG|png|PNG|jpeg|JPEG|proto|PROTO|stl|STL|dae|DAE|obj|OBJ|mp3|MP3|wav|WAV'

asset_urls = []
for path in paths:
    with open(path, 'r') as fd:
        content = fd.read()

    assets = re.findall(rf'({base_url}[^ ]*\.(?:{possible_extensions}))', content)
    asset_urls = list(set(asset_urls + assets))

# create and fill asset folder
if os.path.exists('assets'):
    raise RuntimeError('Error, folder \'assets\' should not exist already but it does')

os.mkdir('assets')
for asset in asset_urls:
    # generate hash of the remote url
    hash = hashlib.sha1(asset.encode('utf-8')).hexdigest()
    # determine location of the file locally
    local_url = asset.replace(rf'https://raw.githubusercontent.com/cyberbotics/webots/{tag}', WEBOTS_HOME)
    # copy to asset folder
    shutil.copyfile(local_url, './assets/' + hash)

# generate zip file
shutil.make_archive('assets', 'zip', 'assets')
shutil.move('assets.zip', WEBOTS_HOME + '/distribution/assets.zip')
shutil.rmtree('assets')
