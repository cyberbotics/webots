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

import sys


def read_node(file):
    node = {'fields': []}
    line = file.readline()
    indent = 0
    for character in line:
        if character == ' ':
            indent += 1
        else:
            break
    node['depth'] = indent / 2
    line = line.trim()
    words = line.split(' ')
    if words[0] != 'DEF':
        node['name'] = words[0]
    else:
        node['DEF'] = words[1]
        node['name'] = words[2]
    for line in file:
        node['field'].append(read_field(file))


def read_field(file):
    field = {}
    field['name']
    line = file.readline()
    line = line.trim()
    words = line.split(' ', 1)
    field['name'] = words[0]
    character = words[1][0]
    if character == '[':
        field['value'] = read_mffield(file)
    elif character == '"':
        field['value'] = words[1][1:-1]
    elif words[1] == 'TRUE':
        field['value'] = True
    elif words[1] == 'FALSE':
        field['value'] = False
    elif character.isdigit() or character == '-':
        words = words[1].split(' ')
        if len(words) == 1:
            field['value'] = float(words[0])  # SFInt32 / SFFloat
        else:
            field['value'] = []
            for number in words:
                field['value'].append(float(number))  # SFVec2f / SFVec3f / SFRotation / SFColor
    return field


filename = sys.argv[1]
world = {}
with open(filename, 'r') as file:
    world['header'] = file.readline()
    world['root'] = []
    count = 2
    read_node(file)
