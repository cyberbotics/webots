#!/usr/bin/env python3.10

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
from generic_distro import get_webots_version

# Missing steps: $(WEBOTS_DISTRO_EXE) $(XVFB)
# 	+@echo "# recompiling controller files"
# 	+@make -s -f Makefile.controllers
# 	+@echo "# checking submodules update"
# 	+@./check_submodules_update.sh

try:
    WEBOTS_HOME = os.getenv('WEBOTS_HOME')
except KeyError:
    sys.exit("WEBOTS_HOME not defined.")

webots_version = get_webots_version()
subprocess.run(["git", "fetch", "--all", "tags"])
tags = subprocess.check_output(["git", "tag", "--points-at", "HEAD"])
if webots_version in tags:
    with open(os.path.join(WEBOTS_HOME, 'resources', 'commit.txt')) as f:
        current_tag = f.readline()
else:
    current_tag = webots_version

# changing URLs in world, proto and controller files"
replace_projects_urls(current_tag)

# generating asset cache
generate_asset_cache(current_tag)

# create distribution
application_name = 'Webots'
if sys.platform == 'win32':
    from copy_msys64_files import copy_msys64_files
    from windows_distro import WindowsWebotsPackage
    print('preparing msys64 folders and files\n')
    copy_msys64_files()
    webots_package = WindowsWebotsPackage(application_name)
elif sys.platform == 'darwin':
    from check_rpath import check_rpath
    from mac_distro import MacWebotsPackage
    print('checking RPATH system\n')
    check_rpath(WEBOTS_HOME)
    webots_package = MacWebotsPackage(application_name)
else:
    from linux_distro import LinuxWebotsPackage
    webots_package = LinuxWebotsPackage(application_name)
print('building distribution source\n')
webots_package.create_webots_bundle()

# revert changes in URLs
replace_projects_urls(current_tag, True)

if sys.platform == 'win32':
    if 'INNO_SETUP_HOME' in os.environ:
        INNO_SETUP_HOME = os.getenv('INNO_SETUP_HOME')
    else:
        INNO_SETUP_HOME = "/C/Program Files (x86)/Inno Setup 6"
    print('creating webots_setup.exe (takes long)\n')
    subprocess.run([INNO_SETUP_HOME + '/iscc', '-Q', 'webots.iss'])
