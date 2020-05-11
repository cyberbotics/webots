#!/usr/bin/env python3

# Copyright 1996-2020 Cyberbotics Ltd.
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

'''Parse Webots world files.'''

import json
import sys

'''This assumes the world file was saved with Webots and the indentation written by Webots was not changed.'''


def read_node(file, line):
    node = {'fields': []}
    words = line.split(' ')
    if words[0] != 'DEF':
        node['name'] = words[0]
    else:
        node['DEF'] = words[1]
        node['name'] = words[2]
    for line in file:
        line = line.strip()
        if line == '}':
            break
        node['fields'].append(read_field(file, line))
    return node


def read_field(file, line):
    field = {}
    words = line.split(' ', 1)
    field['name'] = words[0]
    character = words[1][0]
    if character == '[':
        field['value'] = read_mffield(file)  # MF*
    elif character == '"':
        field['value'] = words[1][1:-1]  # SFString
    elif words[1] == 'TRUE':  # SFBool
        field['value'] = True
    elif words[1] == 'FALSE':  # SFBool
        field['value'] = False
    elif character.isdigit() or character == '-':
        words = words[1].split(' ')
        if len(words) == 1:
            field['value'] = float(words[0])  # SFInt32 / SFFloat
        else:
            field['value'] = []
            for number in words:
                field['value'].append(float(number))  # SFVec2f / SFVec3f / SFRotation / SFColor
    else:
        field['value'] = read_node(file, line)  # SFNode
    return field


def read_mffield(file):
    mffield = []
    while True:
        line = file.readline().strip()
        character = line[0]
        if character == ']':
            break
        if character == '"':
            mffield.append(line[1:-1])  # MFString
        elif line == 'TRUE':
            mffield.append(True)  # MFBool
        elif line == 'FALSE':
            mffield.append(False)  # MFBool
        elif character.isdigit() or character == '-':
            words = line.split(' ')
            if len(words) == 1:
                mffield.append(float(words[0]))  # MFInt32 / MFFloat
            else:
                array = []
                for number in words:
                    array.append(float(number))  # MFVec2f / MFVec3f / MFRotation / MFColor
                mffield.append(array)
        else:
            node = read_node(file, line)
            mffield.append(node)
        words = line.split(' ')
    return mffield


filename = sys.argv[1]
world = {}
with open(filename, 'r') as file:
    world['header'] = file.readline().strip()
    world['root'] = []
    for line in file:
        line = line.strip()
        if line:
            world['root'].append(read_node(file, line))
    print(json.dumps(world, indent=2, sort_keys=True))
