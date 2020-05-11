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


class WebotsWorld:
    '''This class reads a world file and parser its structure.'''
    '''It assumes the world file was saved with Webots and the indentation written by Webots was not changed.'''
    def __init__(self):
        self.content = {}

    def load(self, filename):
        with open(filename, 'r') as self.file:
            self.content['header'] = self.file.readline().strip()
            self.content['root'] = []
            for line in self.file:
                line = line.strip()
                if line:
                    self.content['root'].append(self._read_node(line))

    def save(self, filename):
        self.indentation = 0
        with open(filename, 'w') as self.file:
            self.file.write(self.content['header'] + '\n')
            for node in self.content['root']:
                self._write_node(node)

    def _write_node(self, node):
        if 'DEF' in node:
            name = 'DEF ' + node['DEF'] + ' '
        else:
            name = ''
        name += node['name']
        self.file.write(name + ' {\n')
        self.indentation += 2
        for field in node['fields']:
            self._write_field(field)
        self.indentation -= 2
        self.file.write('}\n')

    def _write_field(self, field):
        line = ' ' * self.indentation
        line += field['name'] + ' '
        value = field['value']
        if isinstance(value, str):
            line += '"' + value + '"'
        elif isinstance(value, (int, float)):
            line += str(value)
        elif isinstance(value, bool):
            line += 'TRUE' if value else 'FALSE'
        elif isinstance(value, list):
            line += '['
            self._write_mf_field(value)
        elif isinstance(value, set):
            self._write_node(value)
        self.file.write(line + '\n')

    def _read_node(self, line):
        node = {'fields': []}
        words = line.split(' ')
        if words[0] != 'DEF':
            node['name'] = words[0]
        else:
            node['DEF'] = words[1]
            node['name'] = words[2]
        for line in self.file:
            line = line.strip()
            if line == '}':
                break
            node['fields'].append(self._read_field(line))
        return node

    def _read_field(self, line):
        field = {}
        words = line.split(' ', 1)
        field['name'] = words[0]
        character = words[1][0]
        if character == '[':
            field['value'] = self._read_mf_field()  # MF*
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
            field['value'] = self._read_node(line)  # SFNode
        return field

    def _read_mf_field(self):
        mffield = []
        while True:
            line = self.file.readline().strip()
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
                node = self._read_node(line)
                mffield.append(node)
            words = line.split(' ')
        return mffield


filename = sys.argv[1]
world = WebotsWorld()
world.load(filename)
world.save("output.wrl")
print(json.dumps(world.content, indent=2, sort_keys=True))
