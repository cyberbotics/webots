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

"""Generate Webots package."""

import os
import sys
import subprocess
from update_urls import replace_projects_urls
from generate_asset_cache import generate_asset_cache
from generate_proto_list import generate_proto_list
from generic_distro import get_webots_version

try:
    WEBOTS_HOME = os.getenv('WEBOTS_HOME')
except KeyError:
    sys.exit("WEBOTS_HOME not defined.")

webots_version = get_webots_version()
subprocess.run(["git", "fetch", "--all", "--tags"])
tags = subprocess.check_output(["git", "tag", "--points-at", "HEAD"]).decode()
if webots_version not in tags:
    commit_file_path = os.path.join(WEBOTS_HOME, 'resources', 'commit.txt')
    if not os.path.exists(commit_file_path):
        subprocess.run(os.path.join(WEBOTS_HOME, 'scripts', 'get_git_info', 'get_git_info.sh'))
    with open(commit_file_path) as f:
        current_tag = f.readline().strip()
else:
    current_tag = webots_version

# changing URLs in world, proto and controller files"
print('# replacing projects urls')
replace_projects_urls(current_tag)

# recompile controllers
print('# recompiling controller files')
with open(os.path.join(WEBOTS_HOME, 'scripts', 'packaging', 'controllers_with_urls.txt')) as f:
    for line in f:
        file_path = line.strip()
        if file_path.endswith('.c'):
            dir_path = os.path.dirname(file_path)
            if dir_path.startswith('/'):
                dir_path = dir_path[1:]
            subprocess.run(['make', '-C', os.path.join(WEBOTS_HOME, dir_path), 'release'])

# generating proto-list.xml
print('# generating proto-list.xml')
generate_proto_list(current_tag)

if sys.platform == 'win32':
    webots_command = 'webots'
else:
    webots_command = os.path.join(WEBOTS_HOME, 'webots')

# sanity check: the following command will display an error if webots fails to start
status = os.system(f'bash -c "{webots_command} --sysinfo"')
if status != 0:
    if sys.platform == 'win32':
        file = os.path.join(WEBOTS_HOME, 'msys64/mingw64/bin/webots-bin.exe')
        os.system(f'ldd {file}')
    else:
        sys.exit("Failed to start webots")

# generating asset cache
print('# generating asset cache')
generate_asset_cache(current_tag, webots_version)

# create distribution
application_name = 'Webots'
if sys.platform == 'win32':
    from windows_distro import WindowsWebotsPackage
    webots_package = WindowsWebotsPackage(application_name)
elif sys.platform == 'darwin':
    from mac_distro import MacWebotsPackage
    webots_package = MacWebotsPackage(application_name)
else:
    from linux_distro import LinuxWebotsPackage
    webots_package = LinuxWebotsPackage(application_name)
print('# generating webots bundle')
webots_package.create_webots_bundle(current_tag != webots_version)
