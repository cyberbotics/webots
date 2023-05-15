#!/usr/bin/env python

# Copyright 1996-2022 Cyberbotics Ltd.
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

import os
import re
import json
from pathlib import Path

# This script generates the FieldModel.js file, which encodes in javascript all the information
# about the nodes normally available in the .wrl files. Every time a change is done to the .wrl
# files, this script must be re-run.

# ensure WEBOTS_HOME is set
if 'WEBOTS_HOME' in os.environ:
    WEBOTS_HOME = os.environ['WEBOTS_HOME']
else:
    raise RuntimeError('Error, WEBOTS_HOME variable is not set.')

SKIPPED_FILES = []

OUTPUT_FILE = os.path.join(WEBOTS_HOME, 'resources', 'web', 'wwi', 'protoVisualizer', 'FieldModel.js')


def list_files():
    files = []
    for root_path, _, file_names in os.walk(os.environ['WEBOTS_HOME']):
        for file in sorted(file_names):
            if file.endswith('.wrl') and file not in SKIPPED_FILES:
                files.append(os.path.join(root_path, file))
    return files


def encode_value(type, value):
    clean_value = value.replace('\"', '\'').strip()

    if type == 'SFBool':
        return clean_value == 'TRUE'
    elif type == 'SFInt32':
        return int(clean_value)
    elif type == 'SFFloat':
        return float(clean_value)
    elif type == 'SFString':
        if clean_value == '\'\'':
            return ''
        return clean_value[1:-1]
    elif type == 'SFVec2f':
        items = clean_value.split(' ')
        if len(items) != 2:
            raise RuntimeError('Decoding of SFVec2f is wrong.')
        return {'x': float(items[0]), 'y': float(items[1])}
    elif type == 'SFVec3f':
        items = clean_value.split(' ')
        if len(items) != 3:
            raise RuntimeError('Decoding of SFVec3f is wrong.')
        return {'x': float(items[0]), 'y': float(items[1]), 'z': float(items[2])}
    elif type == 'SFColor':
        items = clean_value.split(' ')
        if len(items) != 3:
            raise RuntimeError('Decoding of SFColor is wrong.')
        return {'r': float(items[0]), 'g': float(items[1]), 'b': float(items[2])}
    elif type == 'SFColor':
        items = clean_value.split(' ')
        if len(items) != 3:
            raise RuntimeError('Decoding of SFColor is wrong.')
        return {'r': float(items[0]), 'g': float(items[1]), 'b': float(items[2])}
    elif type == 'SFRotation':
        items = clean_value.split(' ')
        if len(items) != 4:
            raise RuntimeError('Decoding of SFRotation is wrong.')
        return {'x': float(items[0]), 'y': float(items[1]), 'z': float(items[2]), 'a': float(items[3])}
    elif type == 'SFNode':
        if clean_value != 'NULL':
            raise RuntimeError('Values other than NULL not supported yet.')
        return None
    elif type.startswith('MF'):
        if clean_value == '[]' or clean_value == '[ ]':
            return []
        # if not empty
        if clean_value.startswith('['):
            clean_value = clean_value[1:-1]
        clean_value = clean_value.split(',')
        items = []
        single_type = 'S' + type[1:]
        for item in clean_value:
            items.append(encode_value(single_type, item))
        return items

    return None


def encode_description(contents):
    description = ''
    for line in contents.split('\n'):
        if not line.startswith('#'):
            break

        description += line[1:].replace("'", '"').strip() + '\\n'
    return description


if __name__ == '__main__':
    files = list_files()

    pattern = re.compile(r"^\s+(field|vrmlField|hiddenField|deprecatedField)"
                         r"\s+([a-zA-Z0-9]+)\s+([a-zA-Z0-9]+)\s+([^\n\#]+)", re.MULTILINE)

    data = {}
    for file in files:
        print(file)
        node_name = Path(file).stem

        data[node_name] = {}
        with open(file, 'r') as f:
            contents = f.read()
            data[node_name]['description'] = encode_description(contents)
            data[node_name]['icon'] = 'https://raw.githubusercontent.com/cyberbotics/webots/released/resources/nodes'\
                                      f'/icons/{node_name}.png'

            data[node_name]['fields'] = {}
            for (vrml_type, type, field_name, default_value) in re.findall(pattern, contents):
                data[node_name]['fields'][field_name] = {}
                data[node_name]['fields'][field_name]['type'] = 'VRML.' + type
                data[node_name]['fields'][field_name]['defaultValue'] = encode_value(type, default_value)

    json_str = json.dumps(data, indent=2)

    replacements = {
        '"VRML.SFBool"': "VRML.SFBool",
        '"VRML.SFInt32"': 'VRML.SFInt32',
        '"VRML.SFFloat"': 'VRML.SFFloat',
        '"VRML.SFString"': 'VRML.SFString',
        '"VRML.SFVec2f"': 'VRML.SFVec2f',
        '"VRML.SFVec3f"': 'VRML.SFVec3f',
        '"VRML.SFColor"': 'VRML.SFColor',
        '"VRML.SFRotation"': 'VRML.SFRotation',
        '"VRML.SFNode"': 'VRML.SFNode',
        '"VRML.MFBool"': "VRML.MFBool",
        '"VRML.MFInt32"': 'VRML.MFInt32',
        '"VRML.MFFloat"': 'VRML.MFFloat',
        '"VRML.MFString"': 'VRML.MFString',
        '"VRML.MFVec2f"': 'VRML.MFVec2f',
        '"VRML.MFVec3f"': 'VRML.MFVec3f',
        '"VRML.MFColor"': 'VRML.MFColor',
        '"VRML.MFRotation"': 'VRML.MFRotation',
        '"VRML.MFNode"': 'VRML.MFNode',
        '\"': '\''
    }

    for old, new in replacements.items():
        json_str = json_str.replace(old, new)

    contents = f'import {{VRML}} from \'./vrml_type.js\';\n\nexport const FieldModel = {json_str};\n'

    with open(OUTPUT_FILE, 'w') as f:
        f.seek(0)
        f.write(contents)
        f.truncate()
