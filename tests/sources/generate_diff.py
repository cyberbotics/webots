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

"""Generate a list of modified files with respect to the parent branch.
This script is used only by Travis."""
import os
import subprocess
import sys

if len(sys.argv) != 2:
    sys.exit('Usage: generate_diff.py parent_branch')
with open(os.path.join(os.getenv('WEBOTS_HOME'), 'tests', 'sources', 'modified_files.txt'), 'w') as file:
    file.write(subprocess.check_output(['git', 'diff', '--name-only', sys.argv[1]]).decode('utf-8'))
