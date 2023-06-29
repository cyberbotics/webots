# Copyright 1996-2023 Cyberbotics Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Sets the environment so that the cache tests can be performed in any condition"""

import os
import subprocess
import sys
import urllib.request
from pathlib import Path

if 'WEBOTS_HOME' in os.environ:
    WEBOTS_HOME = os.environ['WEBOTS_HOME'].replace('\\', '/')
else:
    raise RuntimeError('WEBOTS_HOME environmental variable is not set.')

# necessary in order to be able to run the cache-related tests in the test suite
if 'TESTS_HOME' in os.environ:
    ROOT_FOLDER = os.environ['TESTS_HOME']
else:
    ROOT_FOLDER = WEBOTS_HOME


branch_file_path = os.path.join(WEBOTS_HOME, 'resources', 'branch.txt')
if os.path.exists(branch_file_path):
    with open(branch_file_path, 'r') as file:
        BRANCH = file.read().strip()
elif 'BRANCH_HASH' in os.environ:  # fall-back mechanism for CI built image used by the test_suite
    BRANCH = os.environ['BRANCH_HASH']
else:
    raise RuntimeError('It was not possible to select a branch name. Running the test suite "cache" group may fail.')

test_url = f'https://raw.githubusercontent.com/cyberbotics/webots/{BRANCH}/README.md'
try:
    with urllib.request.urlopen(test_url) as response:
        pass
except urllib.error.HTTPError:
    # This occurs when you are on a branch that does not yet exist in the cyberbotics webots repo (e.g. a local branch).
    print(f'Unable to access {test_url}.')
    repo = "cyberbotics/webots.git"
    print(f'Assuming branch {BRANCH} does not exist in the {repo} repo.')
    print(f'Finding a remote that refers to {repo}.')
    remotes = subprocess.run(["git", "remote", "-v"], capture_output=True, text=True, check=True).stdout
    remote = next(filter(lambda line: repo in line, remotes.splitlines())).split("\t")[0]

    print(f'Finding the nearest ancestor commit that is on either {remote}/develop or {remote}/master.')
    BRANCH = subprocess.run(["git", "merge-base", "HEAD", f'{remote}/develop', f'{remote}/master'],
                            capture_output=True, text=True, check=True).stdout.strip()
    print(f'Using commit {BRANCH}')


def update_cache_urls(revert=False):
    paths = []
    paths.extend((Path(ROOT_FOLDER) / 'tests' / 'cache').rglob('*.proto'))
    paths.extend((Path(ROOT_FOLDER) / 'tests' / 'cache').rglob('*.wbt'))

    for path in paths:
        with open(path, 'r') as fd:
            content = fd.read()

        if revert:
            content = content.replace(ROOT_FOLDER + '/', 'absolute://')
            content = content.replace(f'https://raw.githubusercontent.com/cyberbotics/webots/{BRANCH}/', 'web://')
        else:
            content = content.replace('absolute://', ROOT_FOLDER + '/')
            content = content.replace('web://', f'https://raw.githubusercontent.com/cyberbotics/webots/{BRANCH}/')

        with open(path, 'w', newline='\n') as fd:
            fd.write(content)


if __name__ == '__main__':
    if len(sys.argv) != 2:
        print('Action not provided, options: "setup", "reset"')
    else:
        if sys.argv[1] == "setup":
            update_cache_urls()
        if sys.argv[1] == "reset":
            update_cache_urls(True)
