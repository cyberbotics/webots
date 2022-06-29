#!/usr/bin/env python

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

"""Replace the webots:// URLs with https://raw.githubusercontent.com/cyberbotics/webots/<version>/
   in world, proto and controller files. Replace the webots:// URLs with
   https://cdn.jsdelivr.net/gh/cyberbotics/webots@<version> in robot windows html files."""


import os
import re
import sys
from pathlib import Path


def replace_url(file, tag, github, revert=False):
    if github:
        url = 'https://raw.githubusercontent.com/cyberbotics/webots/'
    else:
        url = 'https://cdn.jsdelivr.net/gh/cyberbotics/webots@'
    with open(file, 'r') as fd:
        content = fd.read()
    if revert:
        # revert any tag
        content = re.sub(url + '[^/]+/', 'webots://', content)
    else:
        content = content.replace('webots://', url + tag + '/')
    with open(file, 'w', newline='\n') as fd:
        fd.write(content)


def replace_projects_urls(tag, revert=False):
    if 'WEBOTS_HOME' in os.environ:
        WEBOTS_HOME = os.environ['WEBOTS_HOME']
    else:
        WEBOTS_HOME = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

    skipped_files = [
      '/projects/samples/howto/url/worlds/url.wbt',
      '/tests/api/worlds/camera_color.wbt'
    ]
    skipped_files = [(Path(WEBOTS_HOME + file)).resolve() for file in skipped_files]

    paths = []
    paths.extend(Path(WEBOTS_HOME + '/projects').rglob('*.proto'))
    paths.extend(Path(WEBOTS_HOME + '/projects').rglob('*.wbt'))
    paths.extend(Path(WEBOTS_HOME + '/tests').rglob('*.wbt'))
    paths.extend(Path(WEBOTS_HOME + '/resources/nodes').rglob('*.wrl'))

    with open(WEBOTS_HOME + '/scripts/packaging/controllers_with_urls.txt', 'r') as files:
        paths.extend(list(map(lambda path: Path(WEBOTS_HOME + '/' + path), files.read().splitlines())))

    for path in paths:
        if path.resolve() not in skipped_files:
            replace_url(path, tag, True, revert)

    paths = []
    paths.extend(Path(WEBOTS_HOME + '/projects').rglob("*/plugins/robot_windows/*/*.html"))

    for path in paths:
        replace_url(path, tag, False, revert)

if __name__ == "__main__":
    if len(sys.argv) != 2:
        sys.exit('Missing argument: commit sha or tag.')

    if sys.argv[1] == 'webots':
        replace_projects_urls(None, True)
    else:
        replace_projects_urls(sys.argv[1])
