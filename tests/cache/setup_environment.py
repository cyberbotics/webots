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
from git import Repo

if 'WEBOTS_HOME' in os.environ:
    WEBOTS_HOME = os.environ['WEBOTS_HOME']
else:
    raise RuntimeError('Error, WEBOTS_HOME variable is not set.')

DIR = os.path.dirname(__file__)


def replace_in_file(file, old, new):
    with open(file, 'r') as f:
        contents = f.read()
        if old not in contents:
            raise RuntimeError(f'String "{old}" could not be found in "{file}".')
        contents = contents.replace(old, new)

    with open(file, 'w') as f:
        f.write(contents)


def setup_cache_environment():
    # do setup for world: local_proto_with_texture.wbt
    file = os.path.join(DIR, 'protos/ShapeWithAbsoluteTexture.proto')
    texture = os.path.join(DIR, 'protos/textures/yellow_texture.jpg')
    replace_in_file(file, 'absolute://tests/cache/protos/textures/yellow_texture.jpg', os.path.abspath(texture))


def clean_cache_environment():
    git = Repo.git
    git.checkout('tests/cache')  # undo all changes done by this script
