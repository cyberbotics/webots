#!/usr/bin/env python3

# Copyright 1996-2020 Cyberbotics Ltd.
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

This list of modified files for testing only the files modified by the current pull request (during the sources tests) and
hence run the CI tests significantly faster.
"""

import json
import os
import subprocess
import sys
import time
from urllib.request import Request, urlopen
from urllib.error import HTTPError


def github_api(request):
    """Send a GitHub API request and return the decoded JSON object."""
    if not request.startswith('https://api.github.com/'):
        request = 'https://api.github.com/' + request
    d = time.time() - github_api.last_time
    if d < 1:
        time.sleep(1 - d)  # wait at least one second between GitHub API calls
    key = os.getenv('GITHUB_API_KEY')
    req = Request(request)
    req.add_header('User-Agent', github_api.user_agent)
    if key is not None:
        req.add_header('Authorization', 'token %s' % key)
    content = ''
    try:
        response = urlopen(req)
        print(response)
        content = response.read()
        print(content)
        content = content.decode()
        print(content)
    except HTTPError as e:
        print(request)
        print(e.reason)
        print(e.info())
    github_api.last_time = time.time()
    return json.loads(content)


if len(sys.argv) == 3:
    commit = sys.argv[1]
    repo = sys.argv[2]
else:
    commit = subprocess.check_output(['git', 'rev-parse', 'HEAD']).decode('utf-8').strip()
    repo = subprocess.check_output(['git', 'config', '--get', 'remote.origin.url']).decode().strip()
    repo = repo[19:-4]  # remove leading 'https://github.com/' and trailing '.git'
github_api.last_time = 0
github_api.user_agent = repo
j = github_api('search/issues?q=' + commit)
filename = os.path.join(os.getenv('WEBOTS_HOME'), 'tests', 'sources', 'modified_files.txt')
if j['total_count'] == 0:  # if no PR is associated with this commit, create an empty modified_files.txt to disable the tests
    open(filename, 'w').close()
else:
    url = j['items'][0]['pull_request']['url']
    j = github_api(url)
    branch = j['base']['ref']
    with open(filename, 'w') as file:
        j = github_api('repos/' + repo + '/compare/' + branch + '...' + commit)
        for f in j['files']:
            if os.path.isfile(f['filename']):  # In case the file has been deleted.
                file.write(f['filename'] + '\n')
