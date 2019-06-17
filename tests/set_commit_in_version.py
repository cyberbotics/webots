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

"""This script should be called when creating the nightly builds to add the commit id to the Webots version."""

import os

path = '..'
if 'WEBOTS_HOME' in os.environ:
    path = os.environ['WEBOTS_HOME']

if len(sys.argv) != 2: # no commit id passed as an argument
    sys.exit('Commit id not passed as argument.')
else:
    commit = sys.argv[1]
    content = ''
    with open(os.path.join(path, 'src', 'webots', 'core', 'WbApplicationInfo.cpp'), 'r') as f:
        content = f.read().replace('R2019b', 'R2019b' + '-commit-' + commit)
    with open(os.path.join(path, 'src', 'webots', 'core', 'WbApplicationInfo.cpp'), 'w') as f:
        f.write(content)
