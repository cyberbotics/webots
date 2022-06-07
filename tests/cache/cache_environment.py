# Copyright 1996-2022 Cyberbotics Ltd.
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

"""Sets the environment so that the cache tests can be performed in any condition"""

import os

if 'WEBOTS_HOME' in os.environ:
    WEBOTS_HOME = os.environ['WEBOTS_HOME']
else:
    raise RuntimeError('Error, WEBOTS_HOME variable is not set.')

SCRIPT_DIRECTORY = os.path.dirname(__file__)

branch_path = os.path.join(WEBOTS_HOME, 'resources', 'branch.txt')
if not os.path.exists(branch_path):
    raise RuntimeError('Error, branch.txt not found. Running the test suite may fail.')

BRANCH = ''
with open(branch_path, 'r') as file:
    BRANCH = file.read().strip()


def generate_action_list(reverse):
    action_list = []

    # setup for world: local_proto_with_texture.wbt
    file = os.path.join(WEBOTS_HOME, 'tests', 'cache', 'protos', 'ShapeWithAbsoluteTexture.proto')
    previous = 'absolute://'
    new = WEBOTS_HOME + '/'
    action_list.append((file, previous, new) if not reverse else (file, new, previous))

    file = os.path.join(WEBOTS_HOME, 'tests', 'cache', 'protos', 'ShapeWithWebTexture.proto')
    previous = 'web://'
    new = f'https://raw.githubusercontent.com/cyberbotics/webots/{BRANCH}/'
    action_list.append((file, previous, new) if not reverse else (file, new, previous))

    # setup for world: web_proto_with_texture.wbt
    file = os.path.join(WEBOTS_HOME, 'tests', 'cache', 'worlds', 'web_proto_with_texture.wbt')
    previous = 'web://'
    new = f'https://raw.githubusercontent.com/cyberbotics/webots/{BRANCH}/'
    action_list.append((file, previous, new) if not reverse else (file, new, previous))
    return action_list


def replace_in_file(file, old, new):
    if not os.path.exists(file):
        raise RuntimeError(f'File "{file}" could not be found.')

    with open(file, 'r') as f:
        contents = f.read()
        if old not in contents:
            raise RuntimeError(f'String "{old}" could not be found in "{file}".')
        contents = contents.replace(old, new)

    with open(file, 'w') as f:
        f.write(contents)


def setup_cache_environment():
    action_list = generate_action_list(reverse=False)

    for action in action_list:
        (file, previous, new) = action
        replace_in_file(file, previous, new)


def reset_cache_environment():
    action_list = generate_action_list(reverse=True)

    for action in action_list:
        (file, previous, new) = action
        replace_in_file(file, previous, new)

#setup_cache_environment()
#reset_cache_environment()