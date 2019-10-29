#!/usr/bin/env python

# Copyright 1996-2019 Cyberbotics Ltd.
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

"""Create a web component scene foreach component of the components.json file."""

import json
import os
from shutil import copyfile

# Paths.
WORLD = 'worlds/web_component_studio.wbt'
TEMPLATE = 'worlds/web_component_studio_template.wbt'
ROBOTS = 'components.json'
WEBOTS_HOME = os.getenv('WEBOTS_HOME')
assert WEBOTS_HOME, 'WEBOTS_HOME is undefined'


def search_and_replace(filename, fromString, toString):
    """Search and replace string in a file."""
    with open(filename, 'r') as file:
        data = file.read()
    data = data.replace(fromString, toString)
    with open(filename, 'w') as file:
        file.write(data)


def run_webots():
    """Run Webots on WORLD with right flags."""
    os.system(WEBOTS_HOME + '/webots --enable-x3d-meta-file-export --mode=fast --minimize ' + WORLD)


# Script logics.
with open(ROBOTS) as f:
    for component in json.load(f)['components']:
        print('Export ' + component['name'] + ' web component...')

        copyfile(TEMPLATE, WORLD)

        search_and_replace(WORLD, '%ROBOT_HEADER%',
                           'Robot { name "%s" children [' % (component['name']) if 'insideRobot' in component else '')
        search_and_replace(WORLD, '%ROBOT_FOOTER%', ']}' if 'insideRobot' in component else '')
        search_and_replace(WORLD, '%ROBOT%', component['proto'])
        search_and_replace(WORLD, '%ROBOT_TRANSLATION%', component['translation'])
        search_and_replace(WORLD, '%ROBOT_ROTATION%', component['rotation'])
        search_and_replace(WORLD, '%ROBOT_NAME%', component['name'])
        search_and_replace(WORLD, '%ROBOT_FIELDS%', component['fields'] if 'fields' in component else '')
        search_and_replace(WORLD, '%VIEWPOINT_POSITION%', component['viewpoint']['position'])
        search_and_replace(WORLD, '%VIEWPOINT_ORIENTATION%', component['viewpoint']['orientation'])

        run_webots()
