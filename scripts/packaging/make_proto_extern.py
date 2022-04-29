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

"""Makes PROTO references extern"""


BASE_URL = 'webots://'
#BASE_URL = 'https://raw.githubusercontent.com/cyberbotics/webots/feature-externproto/'

import os
import sys
from pathlib import Path
from tqdm import tqdm
import re

# def replace_url(file, tag):
#   with open(file, 'r') as fd:
#     content = fd.read()
#   content = content.replace('webots://', 'https://raw.githubusercontent.com/cyberbotics/webots/' + tag + '/')
#   with open(file, 'w', newline='\n') as fd:
#     fd.write(content)


if 'WEBOTS_HOME' in os.environ:
  WEBOTS_HOME = os.environ['WEBOTS_HOME']
else:
  WEBOTS_HOME = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# if len(sys.argv) != 2:
#   sys.exit('Missing argument: commit sha or tag.')
# else:
#   tag = sys.argv[1]
#tag = 'feature-externproto'

protos = []
protos.extend(Path(WEBOTS_HOME + '/projects').rglob('*.proto'))
# paths.extend(Path(WEBOTS_HOME + '/projects').rglob('*.wbt'))
# paths.extend(Path(WEBOTS_HOME + '/tests').rglob('*.wbt'))

known_proto = {}

for proto in protos:
  identifier = os.path.splitext(os.path.basename(proto))[0]
  path = os.path.relpath(proto, os.getcwd())
  known_proto[identifier] = path

assets = []  # worlds and protos alike
# worlds.extend(protos)
assets.extend(Path(WEBOTS_HOME + '/projects').rglob('*.wbt'))
assets.extend(Path(WEBOTS_HOME + '/projects').rglob('*.proto'))

exceptions = ['UsageProfile.proto', 'FourWheelsRobot.proto']

for asset in tqdm(assets):
  if os.path.basename(asset) in exceptions:
    continue

  # open asset
  with open(asset, "r") as f:
    contents = f.read()

  # find any match by bruteforce, to avoid parsing worlds/protos
  matches = []
  for key, value in known_proto.items():
    identifier = key.replace('+', '\+').replace('-', '\-')
    regexp = re.compile(rf'{identifier}\s*' + re.escape('{'))
    if regexp.search(contents):
      matches.append(key)

  # find first non-commented line
  index = None
  contents = contents.splitlines(keepends=True)
  for n, line in enumerate(contents):
    if not line.startswith('#'):
      index = n
      break

  while contents[index].startswith('\n'):
    del contents[index]

  if not index:
    print(f'world {asset} is not valid')
    exit()

  # insert extern proto reference
  #contents = contents.splitlines(keepends=True)
  complete_expr = ""
  for match in matches:
    expr = f'EXTERNPROTO {match} \"{BASE_URL}{known_proto[match]}\"\n'
    if expr not in contents and expr not in complete_expr:
      complete_expr += expr
  contents.insert(index, '\n' + complete_expr + '\n')

  with open(asset, "w") as f:
    contents = "".join(contents)
    f.write(contents)
