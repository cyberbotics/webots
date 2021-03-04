#!/usr/bin/env python

# Copyright 1996-2021 Cyberbotics Ltd.
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

"""Replace the webots:// URLs with https://raw.githubusercontent.com/cyberbotics/webots/<version>/ in world and proto files."""


from pathlib import Path


def replace_url(file, version):
    with open(file, 'r') as fd:
        content = fd.read()
    content = content.replace('webots://', 'https://raw.githubusercontent.com/cyberbotics/webots/' + version + '/')
    with open(file, 'w', newline='\n') as fd:
        fd.write(content)
    return


for path in Path('.').rglob('*.proto'):
    replace_url(path, 'R2021a')
