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
    raise RuntimeError('WEBOTS_HOME environmental variable is not set.')

# necessary in order to be able to run the cache-related tests in the test suite
if 'TESTS_HOME' in os.environ:
    ROOT_FOLDER = os.environ['TESTS_HOME']
else:
    ROOT_FOLDER = WEBOTS_HOME


branch_file_path = os.path.join(WEBOTS_HOME, 'resources', 'branch.txt')
if os.path.exists(branch_file_path):
    with open(branch_file_path, 'r') as file:
        BRANCH = file.read().strip()
elif 'BRANCH_NAME' in os.environ:  # fall-back mechanism for CI built image used by the test_suite
    BRANCH = os.environ['BRANCH_NAME']
else:
    raise RuntimeError('It was not possible to select a branch name. Running the test suite "cache" group may fail.')


def generateActionList(reverse):
    action_list = []

    # setup for world: local_proto_with_texture.wbt & relative?
    file = os.path.join(ROOT_FOLDER, 'tests', 'cache', 'protos', 'ShapeWithAbsoluteTexture.proto')
    previous = 'absolute://'
    new = ROOT_FOLDER + '/'
    action_list.append((file, previous, new) if not reverse else (file, new, previous))

    file = os.path.join(ROOT_FOLDER, 'tests', 'cache', 'protos', 'ShapeWithWebTexture.proto')
    previous = 'web://'
    new = f'https://raw.githubusercontent.com/cyberbotics/webots/{BRANCH}/'
    action_list.append((file, previous, new) if not reverse else (file, new, previous))

    # setup for world: web_proto_with_texture.wbt
    file = os.path.join(ROOT_FOLDER, 'tests', 'cache', 'worlds', 'web_proto_with_texture.wbt')
    previous = 'web://'
    new = f'https://raw.githubusercontent.com/cyberbotics/webots/{BRANCH}/'
    action_list.append((file, previous, new) if not reverse else (file, new, previous))

    # setup for world: absolute_proto_with_texture.wbt
    file = os.path.join(ROOT_FOLDER, 'tests', 'cache', 'worlds', 'absolute_proto_with_texture.wbt')
    previous = 'absolute://'
    new = ROOT_FOLDER + '/'
    action_list.append((file, previous, new) if not reverse else (file, new, previous))

    # setup for world: basenode_with_texture.wbt
    file = os.path.join(ROOT_FOLDER, 'tests', 'cache', 'worlds', 'basenode_with_texture.wbt')
    previous = 'absolute://'
    new = ROOT_FOLDER + '/'
    action_list.append((file, previous, new) if not reverse else (file, new, previous))

    file = os.path.join(ROOT_FOLDER, 'tests', 'cache', 'worlds', 'basenode_with_texture.wbt')
    previous = 'web://'
    new = f'https://raw.githubusercontent.com/cyberbotics/webots/{BRANCH}/'
    action_list.append((file, previous, new) if not reverse else (file, new, previous))

    # setup for world: proto_retrieval_and_import.wbt
    file = os.path.join(ROOT_FOLDER, 'tests', 'cache', 'worlds', 'proto_retrieval_and_import.wbt')
    previous = 'absolute://'
    new = ROOT_FOLDER + '/'
    action_list.append((file, previous, new) if not reverse else (file, new, previous))

    file = os.path.join(ROOT_FOLDER, 'tests', 'cache', 'worlds', 'proto_retrieval_and_import.wbt')
    previous = 'web://'
    new = f'https://raw.githubusercontent.com/cyberbotics/webots/{BRANCH}/'
    action_list.append((file, previous, new) if not reverse else (file, new, previous))

    return action_list


def replaceInFile(file, old, new):
    if not os.path.exists(file):
        raise RuntimeError(f'File "{file}" could not be found.')

    with open(file, 'r') as f:
        contents = f.read()
        if old not in contents:
            raise RuntimeError(f'String "{old}" could not be found in "{file}".')
        contents = contents.replace(old, new)

    with open(file, 'w') as f:
        f.write(contents)


def setupCacheEnvironment():
    action_list = generateActionList(reverse=False)

    for action in action_list:
        (file, previous, new) = action
        replaceInFile(file, previous, new)


def resetCacheEnvironment():
    action_list = generateActionList(reverse=True)

    for action in action_list:
        (file, previous, new) = action
        replaceInFile(file, previous, new)


if __name__ == '__main__':
    setupCacheEnvironment()
    # resetCacheEnvironment()
