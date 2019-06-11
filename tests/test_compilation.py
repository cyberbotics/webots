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

"""Test suite for ROS."""

import glob
import os
import sys
import multiprocessing

from command import Command

path = '..'
if 'WEBOTS_HOME' in os.environ:
    path = os.environ['WEBOTS_HOME']

if 'TRAVIS_COMMIT' in os.environ:
    commit = os.environ['TRAVIS_COMMIT']
    content = ''
    with open(os.path.join(path, 'src', 'webots', 'core', 'WbApplicationInfo.cpp'), 'r') as f:
        content = f.read().replace('R2019b', 'R2019b' + '-commit-' + os.environ['TRAVIS_COMMIT'])
    with open(os.path.join(path, 'src', 'webots', 'core', 'WbApplicationInfo.cpp'), 'w') as f:
        f.write(content)

command = Command('make -C %s distrib -j%d' % (path, multiprocessing.cpu_count()))
command.run(silent=False)
if command.returncode != 0:
    raise RuntimeError('Error when executing the Make command')

if sys.platform.startswith('linux'):
    assert len(glob.glob(path + os.sep + 'distribution' + os.sep + '*')) > 2
else:
    assert len(glob.glob(path + os.sep + 'distribution' + os.sep + '*')) > 1
