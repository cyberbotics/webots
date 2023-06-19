#!/usr/bin/env python3

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

"""Parse Webots world files."""


class WebotsParser:
    """This class reads a world file and parser its structure."""
    """It assumes the world file was saved with Webots and the indentation written by Webots was not changed."""

    def __init__(self):
        self.content = {}
        self.line_count = 0
        self.indentation = 0
        self.file = None

    def load(self, filename):
        with open(filename, 'r') as self.file:
            self.content['header'] = []
            self.content['externprotos'] = []
            self.line_count = 0

            self.content['header'] = []
            while True:
                revert_position = self.file.tell()
                revert_line_count = self.line_count
                line = self.file.readline()
                if line.startswith('#') or not line.strip():
                    self.line_count += 1
                    self.content['header'].append(line.strip())
                elif line.startswith('EXTERNPROTO') or line.startswith('IMPORTABLE EXTERNPROTO'):
                    self.line_count += 1
                    self.content['externprotos'].append(line.strip())
                else:
                    self.file.seek(revert_position)
                    self.line_count = revert_line_count
                    break

            self.content['root'] = []
            for line in self.file:
                line = self._prepare_line(line)
                self.line_count += 1
                if line:
                    if line.startswith('PROTO'):
                        self.content['root'].append(self._read_node_declaration(line))
                    else:
                        self.content['root'].append(self._read_node(line))

    def save(self, filename):
        self.indentation = 0
        with open(filename, 'w', newline='\n') as self.file:
            for header_line in self.content['header']:
                self.file.write(header_line + '\n')
            for externproto_line in self.content['externprotos']:
                self.file.write(externproto_line + '\n')
            for node in self.content['root']:
                if node['type'] == 'node':
                    self._write_node(node)
                else:
                    self._write_node_declaration(node)

    @staticmethod
    def str(value):
        return ('%.15f' % value).rstrip('0').rstrip('.')

    def _write_field_declaration(self, field):
        self.file.write('  ' + field['raw'] + '\n')
        # TODO: Add more fields

    def _write_node_declaration(self, node):
        # Write PROTO header
        self.file.write('PROTO ' + node['name'] + ' [\n')
        for field in node['fields']:
            self._write_field_declaration(field)
        self.file.write(']\n')
        self.file.write('{\n')

        # Write PROTO Robot
        for node in node['root']:
            self.indentation = 2
            self._write_node(node, 2)

        self.file.write('}\n')

    def _write_node(self, node, indentation=0):
        if node is None:
            self.file.write('NULL\n')
            return
        if 'DEF' in node:
            name = 'DEF ' + node['DEF'] + ' '
        elif 'USE' in node:
            self.file.write('USE ' + node['USE'] + '\n')
            return
        else:
            name = indentation * ' '
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
        type_field = field['type']
        if type_field == 'SFString':
            line += '"' + value + '"'
        elif type_field == 'SFInt32' or type_field == 'SFFloat':
            line += value
        elif type_field == 'SFBool':
            line += 'TRUE' if value else 'FALSE'
        elif type_field == 'SFVec2f':
            line += value[0] + ' '
            line += value[1]
        elif type_field == 'SFVec3f' or type_field == 'SFColor':
            line += value[0] + ' '
            line += value[1] + ' '
            line += value[2]
        elif type_field == 'SFRotation':
            line += value[0] + ' '
            line += value[1] + ' '
            line += value[2] + ' '
            line += value[3]
        elif type_field == 'IS':
            line += 'IS '
            line += value
        elif type_field == 'SFNode':
            self.file.write(line)
            self._write_node(value)
            return
        else:  # MF*
            if len(value) == 0:
                line += '[]'
            else:
                self.file.write(line + '[\n')
                self._write_mf_field(type_field, value)
                line = ' ' * self.indentation + ']'
        self.file.write(line + '\n')

    def _write_mf_field(self, type_field, values):
        self.indentation += 2
        indent = ' ' * self.indentation
        count = 0
        self.file.write(indent)
        for index, value in enumerate(values):
            if type_field == 'MFString':
                self.file.write('"' + value + '"\n')
            elif type_field == 'MFInt32' or type_field == 'MFFloat':
                self.file.write(value + ' ')
            elif type_field == 'MFBool':
                self.file.write('TRUE\n' if value else 'FALSE\n')
            elif type_field == 'MFNode':
                self._write_node(value, self.indentation if index > 0 else 0)
            elif type(value) is not list:
                print("Skip {} with values {}".format(type_field, values))
                self.file.write(str(values).replace('[', '').replace("'", '').replace(']', ''))
                break
            elif type_field == 'MFVec2f':
                self.file.write(value[0] + ' ' + value[1])
            elif type_field == 'MFVec3f' or type_field == 'MFColor':
                self.file.write(value[0] + ' ' + value[1] + ' ' + value[2])
            elif type_field == 'MFRotation':
                self.file.write(value[0] + ' ' + value[1] + ' ' + value[2] + ' ' + value[3])
            count += 1
        if type_field in ['MFInt32', 'MFFloat', 'MFVec2f', 'MFVec3f', 'MFColor', 'MFRotation']:
            self.file.write('\n')
        self.indentation -= 2

    def _read_node(self, line):
        node = {'fields': []}
        node['type'] = 'node'
        words = line.split(' ')
        if words[0] == 'DEF':
            node['DEF'] = words[1]
            node['name'] = words[2]
        elif words[0] == 'USE':
            node['USE'] = words[1]
            return node
        else:
            node['name'] = words[0]

        lastWord = words[len(words) - 1]
        if lastWord == '}' or lastWord == '{}':
            return node
        for line in self.file:
            line = self._prepare_line(line)
            self.line_count += 1
            if line.startswith('hidden'):
                print('Removing hidden field: "%s".' % line)
                continue
            elif line == '}':
                break
            node['fields'].append(self._read_field(line))
        return node

    def _read_field_declaration(self, line):
        field = {}
        words = line.split(' ', 4)
        field['name'] = words[2]
        field['type'] = words[1]
        field['raw'] = line
        if field['type'] == 'SFVec3f':
            field['value'] = words[3].split(' ')[:3]
        elif field['type'] == 'SFRotation':
            field['value'] = words[3].split(' ')[:3]
        # TODO: Add more fields

        return field

    def _read_node_declaration(self, line):
        node = {'fields': []}
        words = line.split(' ')
        node['name'] = words[1]
        node['type'] = 'proto'

        # Read fields
        for line in self.file:
            line = self._prepare_line(line)
            self.line_count += 1
            if line == ']':
                break
            node['fields'].append(self._read_field_declaration(line))

        # Read subnodes
        node['root'] = []
        for line in self.file:
            line = self._prepare_line(line)
            self.line_count += 1
            if line == '}':
                break
            if line != '{':
                node['root'].append(self._read_node(line))

        return node

    def _prepare_line(self, line):
        if '%<' in line or '>%' in line or '%{' in line or '}%' in line:
            raise Exception(
                f'JavaScript fragment found at line {self.line_count}. This script cannot handle JavaScript fragments.')
        return line.split('#')[0].strip()

    def _read_field(self, line):
        field = {}
        words = line.split(' ', 1)
        field['name'] = words[0]
        if len(words) < 2:
            raise Exception(f'Line: {self.line_count}. Expecting more than a single word: `{words}` in line')
        character = words[1][0]
        if character == '[':
            if len(words[1]) > 1 and words[1][1] == ']':  # empty MF field
                field['type'] = 'MF'
                field['value'] = []
            else:
                first_line = words[1].lstrip('[ ') if len(words[1]) > 2 else None
                field['type'], field['value'] = self._read_mf_field(first_line=first_line)  # MF*
        elif character == '"':
            field['type'] = 'SFString'
            field['value'] = words[1][1:-1]
        elif ' IS ' in line:
            field['type'] = 'IS'
            field['value'] = line.split(' IS ')[1]
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

    def _read_mf_field(self, first_line=None):
        mffield = []
        type_field = ''
        should_finish = False
        while not should_finish:
            line = None
            if first_line:
                line = first_line
                first_line = None
            else:
                line = self.file.readline().strip()
                self.line_count += 1
            if not line:
                continue
            character = line[0]
            if character == ']':
                break
            if character == '"':
                type_field = 'MFString'
                mffield.append(line[1:-1])  # MFString
            elif line == 'TRUE':
                type_field = 'MFBool'
                mffield.append(True)  # MFBool
            elif line == 'FALSE':
                type_field = 'MFBool'
                mffield.append(False)  # MFBool
            elif character.isdigit() or character == '-':
                groups = line.split(',')
                elements = []
                for group in groups:
                    elements = elements + [x for x in group.split(' ') if x]
                if elements[-1] == ']':
                    should_finish = True
                    elements = elements[:-1]

                # Parse MFVec2f / MFVec3f / MFRotation / MFColor
                length = len(elements)
                if length == 2:
                    type_field = 'MFVec2f'
                elif length == 3:
                    type_field = 'MFVec3f'  # FIXME: could be MFColor as well
                elif length == 4:
                    type_field = 'MFRotation'
                else:
                    type_field = 'MFFloat' if ',' in line else 'MFInt32'
                mffield.extend(elements)
            else:
                type_field = 'MFNode'
                node = self._read_node(line)
                mffield.append(node)
        if len(mffield) > 4:
            if isinstance(mffield[0], dict):
                type_field = 'MFNode'
            else:
                type_field = 'MFFloat' if ',' in line else 'MFInt32'

        return type_field, mffield
