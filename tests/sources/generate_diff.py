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

"""Generate a list of modified files with respect to the parent branch. If not passed as an argument, the
parent branch is automatically computed. This list of modified files is used by Travis for testing only the
files modified by the current pull request (during the sources tests) and hence run the CI tests
significantly faster."""

import git
import json
import os
import subprocess
import sys
import urllib.request
from urllib.error import HTTPError


def json_wget(url):
    key = os.getenv('TRAVIS_GITHUB_API_KEY')
    req = urllib.request.Request(url)
    req.add_header('User-Agent', 'omichel')
    if key is not None:
        req.add_header('Authorization', 'token %s' % key)
    try:
        response = urllib.request.urlopen(url)
    except HTTPError as e:
        print(e.reason)
        print(e.info())
    content = response.read()
    return json.loads(content)


curdir = os.getcwd()
os.chdir(os.getenv('WEBOTS_HOME'))
if len(sys.argv) != 2:  # no parent branch passed as an argument, computing it from GitHub API
    commit = os.getenv("TRAVIS_COMMIT")
    if commit is None:
        commit = subprocess.check_output(['git', 'rev-parse', 'head']).decode('utf-8').strip()
    print(commit)
    j = json_wget('https://api.github.com/search/issues?q=' + commit)
    url = j["items"][0]["pull_request"]["url"]
    print(url)
    j = json_wget(url)
    branch = j["base"]["ref"]
else:
    branch = sys.argv[1]
# branch = 'origin/' + branch
with open(os.path.join(os.getenv('WEBOTS_HOME'), 'tests', 'sources', 'modified_files.txt'), 'w') as file:
    #  current_branch = subprocess.check_output(['git', 'rev-parse', '--abbrev-ref', 'HEAD']).decode()
    #  subprocess.check_output(['git', 'fetch'])
    #  subprocess.check_output(['git', 'checkout', branch])
    #  subprocess.check_output(['git', 'checkout', current_branch])
    file.write(subprocess.check_output("git diff --name-only " + branch, shell=True).decode())
os.chdir(curdir)
