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

"""Parse Webots world files."""

import sys


class WebotsParser:
    """This class reads a world file and parser its structure."""
    """It assumes the world file was saved with Webots and the indentation written by Webots was not changed."""
    def __init__(self):
        self.content = {}

    def load(self, filename):
        with open(filename, 'r') as self.file:
            self.content['header'] = self.file.readline().strip()
            self.line_count = 1
            self.content['root'] = []
            for line in self.file:
                line = line.strip()
                self.line_count += 1
                if line:
                    self.content['root'].append(self._read_node(line))

    def save(self, filename):
        self.indentation = 0
        with open(filename, 'w', newline='\n') as self.file:
            self.file.write(self.content['header'] + '\n')
            for node in self.content['root']:
                self._write_node(node)

    @staticmethod
    def str(value):
        return ('%.15f' % value).rstrip('0').rstrip('.')

    def _write_node(self, node):
        if node is None:
            self.file.write('NULL\n')
            return
        if 'DEF' in node:
            name = 'DEF ' + node['DEF'] + ' '
        elif 'USE' in node:
            self.file.write('USE ' + node['USE'] + '\n')
            return
        else:
            name = ''
        name += node['name']
        self.file.write(name + ' {\n')
        self.indentation += 2
        for field in node['fields']:
            self._write_field(field)
        self.indentation -= 2
        self.file.write(' ' * self.indentation + '}\n')

    def _write_field(self, field):
        line = ' ' * self.indentation
        line += field['name'] + ' '
        value = field['value']
        type = field['type']
        if type == 'SFString':
            line += '"' + value + '"'
        elif type == 'SFInt32' or type == 'SFFloat':
            line += value
        elif type == 'SFBool':
            line += 'TRUE' if value else 'FALSE'
        elif type == 'SFVec2f':
            line += value[0] + ' '
            line += value[1]
        elif type == 'SFVec3f' or type == 'SFColor':
            line += value[0] + ' '
            line += value[1] + ' '
            line += value[2]
        elif type == 'SFRotation':
            line += value[0] + ' '
            line += value[1] + ' '
            line += value[2] + ' '
            line += value[3]
        elif type == 'SFNode':
            self.file.write(line)
            self._write_node(value)
            return
        else:  # MF*
            if len(value) == 0:
                line += '[]'
            else:
                self.file.write(line + '[\n')
                self._write_mf_field(type, value)
                line = ' ' * self.indentation + ']'
        self.file.write(line + '\n')

    def _write_mf_field(self, type, values):
        self.indentation += 2
        indent = ' ' * (self.indentation)
        count = 0
        smallSeparator = True if len(values) > 25 else False
        for value in values:
            if type in ['MFInt32', 'MFFloat', 'MFVec2f', 'MFVec3f', 'MFColor', 'MFRotation']:
                if count == 0:
                    self.file.write(indent)
                elif smallSeparator or (type in ['MFInt32', 'MFFloat'] and count % 10 != 0):
                    self.file.write(', ')
                else:
                    self.file.write('\n')
                    self.file.write(indent)
            else:
                self.file.write(indent)
            if type == 'MFString':
                self.file.write('"' + value + '"\n')
            elif type == 'MFInt32' or type == 'MFFloat':
                self.file.write(value)
            elif type == 'MFBool':
                self.file.write('TRUE\n' if value else 'FALSE\n')
            elif type == 'MFVec2f':
                self.file.write(value[0] + ' ' + value[1])
            elif type == 'MFVec3f' or type == 'MFColor':
                self.file.write(value[0] + ' ' + value[1] + ' ' + value[2])
            elif type == 'MFRotation':
                self.file.write(value[0] + ' ' + value[1] + ' ' + value[2] + ' ' + value[3])
            elif type == 'MFNode':
                self._write_node(value)
            count += 1
        if type in ['MFInt32', 'MFFloat', 'MFVec2f', 'MFVec3f', 'MFColor', 'MFRotation']:
            self.file.write('\n')
        self.indentation -= 2

    def _read_node(self, line):
        node = {'fields': []}
        words = line.split(' ')
        if words[0] == 'DEF':
            node['DEF'] = words[1]
            node['name'] = words[2]
        elif words[0] == 'USE':
            node['USE'] = words[1]
            return node
        else:
            node['name'] = words[0]
        for line in self.file:
            line = line.strip()
            self.line_count += 1
            if line.startswith('hidden'):
                print('Removing hidden field: "%s".' % line)
                continue
            elif line == '}':
                break
            node['fields'].append(self._read_field(line))
        return node

    def _read_field(self, line):
        field = {}
        words = line.split(' ', 1)
        field['name'] = words[0]
        if len(words) < 2:
            sys.exit('Line:', self.line_count, 'Expecting more than a single word:', words)
        character = words[1][0]
        if character == '[':
            if len(words[1]) > 1 and words[1][1] == ']':  # empty MF field
                field['type'] = 'MF'
                field['value'] = []
            else:
                (field['type'], field['value']) = self._read_mf_field()  # MF*
        elif character == '"':
            field['type'] = 'SFString'
            field['value'] = words[1][1:-1]
        elif words[1] == 'TRUE':
            field['type'] = 'SFBool'
            field['value'] = True
        elif words[1] == 'FALSE':
            field['type'] = 'SFBool'
            field['value'] = False
        elif words[1] == 'NULL':
            field['type'] = 'SFNode'
            field['value'] = None
        elif character.isdigit() or character == '-':
            words = words[1].split(' ')
            length = len(words)
            if length == 1:
                if '.' in words[0]:
                    field['type'] = 'SFFloat'
                    field['value'] = words[0]
                else:
                    field['type'] = 'SFInt32'  # FIXME: this may be wrong! But it would be very complicated to the correct value
                    field['value'] = words[0]
            else:
                field['value'] = []
                if length == 2:
                    field['type'] = 'SFVec2f'
                elif length == 3:
                    field['type'] = 'SFVec3f'  # FIXME: this may be wrong (could be SFColor), but difficult to determine
                elif length == 4:
                    field['type'] = 'SFRotation'
                for number in words:
                    field['value'].append(number)
        else:
            field['type'] = 'SFNode'
            field['value'] = self._read_node(words[1])
        return field

    def _read_mf_field(self):
        mffield = []
        type = ''
        while True:
            line = self.file.readline().strip()
            self.line_count += 1
            character = line[0]
            if character == ']':
                break
            if character == '"':
                type = 'MFString'
                mffield.append(line[1:-1])  # MFString
            elif line == 'TRUE':
                type = 'MFBool'
                mffield.append(True)  # MFBool
            elif line == 'FALSE':
                type = 'MFBool'
                mffield.append(False)  # MFBool
            elif character.isdigit() or character == '-':
                groups = line.split(',')
                for group in groups:
                    if group is None:
                        continue
                    words = group.strip().split(' ')
                    length = len(words)
                    if length == 1 or ',' in words[0]:
                        if not type:
                            type = 'MFInt32'
                        for value in words:
                            if value.endswith(','):
                                value = value[:-1]
                            if '.' in value:
                                type = 'MFFloat'
                            mffield.append(value)
                    else:
                        array = []
                        if length == 2 or ',' in words[1]:
                            type = 'MFVec2f'
                        elif length == 3 or ',' in words[2]:
                            type = 'MFVec3f'  # FIXME: could be MFColor as well
                        elif length == 4 or ',' in words[3]:
                            type = 'MFRotation'
                        for number in words:
                            array.append(number)  # MFVec2f / MFVec3f / MFRotation / MFColor
                        mffield.append(array)
            else:
                type = 'MFNode'
                node = self._read_node(line)
                mffield.append(node)
            words = line.split(' ')
        return (type, mffield)
