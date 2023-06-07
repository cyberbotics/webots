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

'''Format PROTO indentations.'''

import sys


def indent_vrml_file(file_path):
    indent_spaces = 2  # Number of spaces for each indentation level
    indentation_level = 0  # Current indentation level
    indent_next_line = 0  # If we need to indent the next line in the case of a if without {}
    js_indentation_level = 0

    with open(file_path, 'r') as file:
        lines = file.readlines()

    indented_lines = []

    in_js = False
    for line in lines:
        stripped_line = line.strip()
        comment_index = line.find('#')
        if comment_index != -1:
            line = line[:comment_index]
        comparison_line = line.strip()

        if stripped_line:
            # ignore templating lines
            if comparison_line.startswith('%<'):
                in_js = True

            if comparison_line.endswith('>%'):
                in_js = False
                js_indentation_level = 0

            # Check if the line ends a new block
            if (comparison_line.startswith('}') or comparison_line.startswith(']')):
                if in_js:
                    js_indentation_level -= 1
                else:
                    indentation_level -= 1

            # Add the indented line to the list
            current_indentation_level = indentation_level if not in_js else indentation_level + js_indentation_level
            indented_line = ' ' * (indent_spaces * (current_indentation_level + indent_next_line)) + stripped_line
            indent_next_line = 0
            indented_lines.append(indented_line)

            # Check if the line starts a block
            if (comparison_line.endswith('{') or comparison_line.endswith('[')):
                if in_js:
                    js_indentation_level += 1
                else:
                    indentation_level += 1
            # case of if/else without {}
            elif comparison_line.startswith('if') or comparison_line.startswith('else') or comparison_line.startswith('elif'):
                indent_next_line = 1
        else:
            indented_lines.append('')
    with open(file_path, 'w') as file:
        file.write('\n'.join(indented_lines) + '\n')


if __name__ == "__main__":
    # Retrieve the file path from the command-line argument
    file_path = sys.argv[1]
    indent_vrml_file(file_path)
