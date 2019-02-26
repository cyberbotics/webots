"""Create a web component scene foreach component of the components.json file."""

# Usage:
# `python web_component_studio.py`

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
for component in json.load(open(ROBOTS))['components']:
    print ('Export ' + component['name'] + ' web component...')

    copyfile(TEMPLATE, WORLD)

    search_and_replace(WORLD, '%ROBOT%', component['proto'])
    search_and_replace(WORLD, '%ROBOT_TRANSLATION%', component['translation'])
    search_and_replace(WORLD, '%ROBOT_ROTATION%', component['rotation'])
    search_and_replace(WORLD, '%ROBOT_NAME%', component['name'])
    search_and_replace(WORLD, '%VIEWPOINT_POSITION%', component['viewpoint']['position'])
    search_and_replace(WORLD, '%VIEWPOINT_ORIENTATION%', component['viewpoint']['orientation'])

    run_webots()
