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
import multiprocessing

from command import Command

path = '..'
if 'WEBOTS_HOME' in os.environ:
    path = os.environ['WEBOTS_HOME']

#if 'TRAVIS_EVENT_TYPE' in os.environ and os.environ['TRAVIS_EVENT_TYPE'] == 'cron':
command = Command('make -C %s distrib -j%d' % (path, multiprocessing.cpu_count()))
command.run(silent=False)
if command.returncode != 0:
    raise RuntimeError('Error when executing the Make command')

assert len(glob.glob(path + os.sep + 'distribution') > 2)
