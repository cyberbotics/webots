#!/usr/bin/env python

# Copyright 1996-2023 Cyberbotics Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Makes local PROTO references extern to fix the compatibility between R2022a and R2022b"""


import os
import re
import sys
import argparse
from pathlib import Path
import xml.etree.ElementTree as ET

parser = argparse.ArgumentParser(description='Declares EXTERNPROTO references')
parser.add_argument('webots_path', nargs=1, default=None)
args = parser.parse_args()

# ensure it's a valid webots path
if sys.platform.startswith('darwin'):
    protolist = os.path.join(args.webots_path[0], 'contents', 'resources', 'proto-list.xml')
else:
    protolist = os.path.join(args.webots_path[0], 'resources', 'proto-list.xml')

if not os.path.exists(protolist):
    raise RuntimeError(f'Path {protolist} is not a valid webots path, proto-list.xml not found')

# parse proto-list.xml
tree = ET.parse(protolist)
root = tree.getroot()

official_protos = {}
for proto in root:
    official_protos[proto.find('name').text] = proto.find('url').text

files = []
files.extend(Path('./protos').rglob('*.proto'))
files.extend(Path('./worlds').rglob('*.wbt'))

local_files = {}
for file in files:
    local_files[file.stem] = file.resolve()

for file, path in local_files.items():
    # read asset
    with open(path, "r") as f:
        contents = f.read()

    # find any match by bruteforce, starting with local ones
    local_matches = []
    for key, value in local_files.items():
        identifier = key.replace('+', r'\+').replace('-', r'\-').replace('_', r'\_')
        regexp = re.compile(rf'{identifier}\s*' + re.escape('{'))
        if regexp.search(contents):
            if key not in local_matches:
                local_matches.append(key)

    # now among the official ones
    official_matches = []
    for key, value in official_protos.items():
        identifier = key.replace('+', r'\+').replace('-', r'\-').replace('_', r'\_')
        regexp = re.compile(rf'{identifier}\s*' + re.escape('{'))
        if regexp.search(contents):
            if key not in official_matches:
                official_matches.append(key)

    contents = contents.splitlines(keepends=True)

    header = []
    index = None
    for n, line in enumerate(contents):
        if line.replace(' ', '').replace('\t', '') == '\n':
            continue
        clean_line = line.strip()
        if clean_line.startswith('#'):
            header.append(clean_line + '\n')
        else:
            index = n
            break

    if not header:
        raise RuntimeError(f'File {path.name} is invalid because it has no header')
    version = re.search(r'^#\s*VRML_SIM\s+([a-zA-Z0-9\-]+)\s+utf8', header[0])
    if not version:
        raise RuntimeError(f'The header of {path.name} is not recognized')
    elif (version.group(1) >= 'R2022b'):
        print(f'Skipping "{path.name}" because header is already R2022b or higher')
        continue

    if index:
        # consume all the empty lines following the index or previous attempts at declaring EXTERNPROTO
        while contents[index] == '\n' or contents[index].startswith('EXTERNPROTO'):
            del contents[index]
    else:
        raise RuntimeError(f'File {proto} has an invalid structure')

    if len(local_matches) > 0 or len(official_matches) > 0:
        print(f'File "{path.name}" depends on:')
        for match in local_matches:
            print(f'  [LOCAL] {match}.proto')
        for match in official_matches:
            if match not in local_matches:
                print(f'  [OFFICIAL] {match}.proto')
    else:
        print(f'File "{path.name}" does not reference any PROTO')

    declarations = []
    for match in local_matches:
        relative_path = os.path.relpath(local_files[match], path.parent)
        relative_path = relative_path.replace("\\", "/")
        declarations.append(f'EXTERNPROTO "{relative_path}"\n')

    for match in official_matches:
        if match in local_matches:  # favor existing local PROTO over official one if the name is the same
            continue

        declarations.append(f'EXTERNPROTO "{official_protos[match]}"\n')

    # insert declaration in the PROTO file
    if len(declarations) > 0:
        contents.insert(index, '\n')
        for declaration in declarations:
            contents.insert(index, declaration)
        contents.insert(index, '\n')

    # update proto header
    contents = contents[n:]  # remove old header
    header[0] = '#VRML_SIM R2022b utf8\n'
    contents = header + contents

    # write to file
    with open(path, "w") as f:
        contents = "".join(contents)
        f.write(contents)
