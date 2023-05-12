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

"""Convert Transforms with no scale or the default one into Poses"""

# usage: put the script in the root folder of the project you want to convert and run it.
from webots_parser import WebotsParser
from pathlib import Path

def convert_to_enu(filename):
    world = WebotsParser()
    world.load(filename)

    print('Scan for transforms')
    if filename.endswith('.proto'):
        convert_children(world.content['root'][0]['root'][0])
    else:
        for node in world.content['root']:
            convert_children(node)

    world.save(filename)
    print('Conversion done')

def convert_children(node):
    if 'USE' in node:
        return
    if node['name'] == 'Transform':
        canConvert = True
        for field in node['fields']:
            if field['name'] == 'scale':
                if field['value'] == ['1', '1', '1']:
                    node['fields'].remove(field)
                else:
                    print('Transform found but not replaced due to a non-default scale')
                    canConvert = False

        if canConvert:
            print('Transform found and replaced')
            node['name'] = 'Pose'

    for field in node['fields']:
        if field['name'] in 'children':
            for child in field['value']:
                convert_children(child)
        elif field['name'] in 'endPoint':
            convert_children(field['value'])
        elif field['name'] in 'boundingObject':
            convert_children(field['value'])

if __name__ == "__main__":
    files = []
    files.extend(Path('./protos').rglob('*.proto'))
    files.extend(Path('./worlds').rglob('*.wbt'))

    local_files = {}
    for file in files:
        local_files[file.stem] = file.resolve()

    for file, path in local_files.items():
        print('Loading ' + str(path))
        convert_to_enu(str(path))
