"""Create a robot component scene foreach robot of the robots.json file."""

# Usage:
# `python robot_component_studio.py`

import json
import os
from shutil import copyfile

# Paths.
WORLD = 'worlds/robot_component_studio.wbt'
TEMPLATE = 'worlds/robot_component_studio_template.wbt'
ROBOTS = 'robots.json'
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
for robot in json.load(open(ROBOTS))['robots']:
    print ('Export ' + robot['name'] + ' robot component...')

    copyfile(TEMPLATE, WORLD)

    search_and_replace(WORLD, '%ROBOT%', robot['proto'])
    search_and_replace(WORLD, '%ROBOT_TRANSLATION%', robot['translation'])
    search_and_replace(WORLD, '%ROBOT_ROTATION%', robot['rotation'])
    search_and_replace(WORLD, '%ROBOT_NAME%', robot['name'])
    search_and_replace(WORLD, '%VIEWPOINT_POSITION%', robot['viewpoint']['position'])
    search_and_replace(WORLD, '%VIEWPOINT_ORIENTATION%', robot['viewpoint']['orientation'])

    run_webots()
