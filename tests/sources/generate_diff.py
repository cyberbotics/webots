#!/usr/bin/env python3

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

"""Generate a list of modified files with respect to the parent branch. This list of modified files is used by Travis for
testing only the files modified by the current pull request (during the sources tests) and hence run the CI tests
significantly faster."""

import json
import os
import subprocess
import time
import urllib.request
from urllib.error import HTTPError


def github_api(url):
    d = time.time() - github_api.last_time
    if d < 1:
        time.sleep(1 - d)  # wait at least one second between GitHub API calls
    key = os.getenv('GITHUB_API_KEY')
    req = urllib.request.Request(url)
    req.add_header('User-Agent', 'omichel/webots')
    if key is not None:
        req.add_header('Authorization', 'token %s' % key)
    try:
        response = urllib.request.urlopen(url)
    except HTTPError as e:
        print(e.reason)
        print(e.info())
    content = response.read()
    github_api.last_time = time.time()
    return json.loads(content)


github_api.last_time = 0
commit = os.getenv("TRAVIS_COMMIT")
if commit is None:
    commit = subprocess.check_output(['git', 'rev-parse', 'head']).decode('utf-8').strip()
j = github_api('https://api.github.com/search/issues?q=' + commit)
url = j["items"][0]["pull_request"]["url"]
j = github_api(url)
branch = j["base"]["ref"]
with open(os.path.join(os.getenv('WEBOTS_HOME'), 'tests', 'sources', 'modified_files.txt'), 'w') as file:
    j = github_api('https://api.github.com/repos/omichel/webots/compare/' + branch + '...' + commit)
    for f in j['files']:
        file.write(f['filename'] + '\n')
