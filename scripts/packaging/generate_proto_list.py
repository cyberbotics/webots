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

"""Generate known proto list"""
import os
from pathlib import Path
import sys
import shutil
import hashlib
from datetime import date

SKIPPED_DIRECTORIES = []

# ensure WEBOTS_HOME is set and tag was provided
if 'WEBOTS_HOME' in os.environ:
    WEBOTS_HOME = os.environ['WEBOTS_HOME']
else:
    raise RuntimeError('Error, WEBOTS_HOME variable is not set.')

if 'WEBOTS_DEVELOPMENT_ENVIRONMENT' in os.environ:
    base_url = "webots://"
else:
    base_url = "https://raw.githubusercontent.com/cyberbotics/webots/"

if len(sys.argv) != 2:
    sys.exit('Missing argument: commit sha or tag.')
else:
    tag = sys.argv[1]

with open(os.path.join(WEBOTS_HOME, 'resources', 'version.txt'), 'r') as file:
    filename = f'{WEBOTS_HOME}/resources/proto-list-{file.readline().strip()}.txt'

# retrieve all proto
assets = []
assets.extend(Path(WEBOTS_HOME + '/projects').rglob('*.proto'))

if (os.path.exists(filename)):
  os.remove(filename)

with open(filename, 'w') as file:
    for proto in assets:
        if any(x in str(proto) for x in SKIPPED_DIRECTORIES):
            continue
        # get node name from file name
        node_name = proto.stem
        # generate remote url
        remote_url = str(proto).replace(WEBOTS_HOME, rf'{base_url}{tag}')
        # add to list
        file.write(f'{node_name} {remote_url}\n')

