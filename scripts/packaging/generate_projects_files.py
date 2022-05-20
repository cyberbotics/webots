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

"""Generate and output the list of projects files to be included in the release package."""


import os
import sys
import fnmatch
import re
if sys.platform == 'linux':
    import distro  # needed to retrieve the Ubuntu version

# add all the files from the projects folder except:
# .gitignore
# OS generated files
# text editor backup files
# build folders


def check_exist_in_projects(files):
    """Check if the given files exist in the projects directory."""
    valid_environment = True
    for file in files:
        if not os.path.exists("../../" + file):
            sys.stderr.write("\x1B[31mFile or folder not found: " + file + "\x1b[0m\n")
            valid_environment = False
    return valid_environment


if sys.platform == 'win32':
    dll_extension = '.dll'
    platform = 'windows'
elif sys.platform == 'darwin':
    dll_extension = '.dylib'
    platform = 'mac'
else:
    dll_extension = '.so'
    platform = 'linux'

with open("omit_in_projects.txt") as f:
    omit_in_projects = f.read().splitlines()

with open("omit_in_projects_" + platform + ".txt") as f:
    omit_in_projects += f.read().splitlines()

if 'SNAPCRAFT_PROJECT_NAME' in os.environ:
    with open("omit_in_projects_snap.txt") as f:
        omit_in_projects += f.read().splitlines()

with open("recurse_in_projects.txt") as f:
    recurse_in_projects = f.read().splitlines()
valid_environment = check_exist_in_projects(recurse_in_projects)

with open("exist_in_projects.txt") as f:
    exist_in_projects = f.read().splitlines()
exist_in_projects_platform_path = "exist_in_projects"
if sys.platform == 'linux':
    exist_in_projects_platform_path += "_" + platform + '_' + distro.version()
exist_in_projects_platform_path += ".txt"
with open(exist_in_projects_platform_path) as f:
    exist_in_projects += f.read().splitlines()
valid_environment = check_exist_in_projects(exist_in_projects) & valid_environment

if not valid_environment:
    sys.exit(-1)

os.chdir('../..')


def is_ignored_file(f):
    """Check if this file has to be ignored.

    Ignored files includes:
    - .gitignore
    - OS generated files
    - text editor backup files
    - XCF image files (Gimp)
    """
    return f == '.gitignore' or f == '.DS_Store' or f == '.DS_Store?' or \
        f == '.Spotlight-V100' or f == '.Trashes' or f == 'ehthumbs.db' or f == 'Thumbs.db' or \
        f.startswith("._") or f.endswith(".swp") or f.endswith(".bak") or f.endswith("~") or \
        f.endswith(".xcf")


def omit_match(f):
    """Check if current file or directory has to be omitted.

    Files and directories to be omitted are listed in the folling files:
    - omit_in_projects.txt
    - omit_in_projects_linux.txt
    - omit_in_projects_mac.txt
    - omit_in_projects_windows.txt
    - omit_in_projects_snap.txt
    """
    for fn in omit_in_projects:
        if fn.endswith('/') and (f + '/').startswith(fn):
            return True
        if fnmatch.fnmatch(f, fn):
            return True
    return False


def list_folder(p):
    """List files to be released located in the current directory or in subdirectories.

    Skip .gitignore and build folders, mark EXE and DLL appropriately.
    """
    print(p + '/')
    for f in os.listdir(p):
        pf = p + '/' + f
        if omit_match(pf):
            continue
        if os.path.isfile(pf):
            if is_ignored_file(f):
                continue
            if sys.platform == 'win32' and f.endswith('.exe'):
                print(p + '/' + os.path.splitext(f)[0] + ' [exe]')
            elif f.endswith(dll_extension):
                lib = os.path.splitext(f)[0]
                if sys.platform != 'win32' and lib.startswith('lib'):
                    lib = lib[3:]  # remove the 'lib' prefix on Linux and macOS
                print(p + '/' + lib + ' [dll]')
            elif sys.platform != 'win32' and os.access(pf, os.X_OK) and f.find('.') == -1:
                print(pf + ' [exe]')
            else:
                print(pf)
        else:
            if f == 'build' or f == 'com':
                continue  # skip any build or com folder
            elif pf in recurse_in_projects:
                print(pf + " [recurse]")
                continue
            list_folder(pf)  # recurse


def list_controller(p):
    """Print the absolute path of controllers files to be released located in the current directory."""
    print(p + '/')
    for f in os.listdir(p):
        pf = p + '/' + f
        if omit_match(pf):
            continue
        if os.path.isfile(pf):
            if is_ignored_file(f):
                continue
            if sys.platform == 'win32' and f.endswith('.exe'):
                print(p + '/' + os.path.splitext(f)[0] + ' [exe]')
            elif sys.platform != 'win32' and os.access(pf, os.X_OK) and f.find('.') == -1:
                print(pf + ' [exe]')
            else:
                print(pf)
        else:
            if f == 'build' or f == 'com':
                continue
            elif pf in recurse_in_projects:
                print(pf + " [recurse]")
                continue
            list_folder(pf)


def list_controllers(p):
    """List valid controllers files and directory."""
    print(p + '/')
    for f in os.listdir(p):
        pf = p + '/' + f
        if omit_match(pf):
            continue
        if os.path.isfile(pf):
            if is_ignored_file(f):
                continue
            print(pf)
        else:
            list_controller(pf)


def list_plugins(p):
    """List valid plugins files and directories."""
    print(p + '/')
    for f in os.listdir(p):
        pf = p + '/' + f
        if omit_match(pf):
            continue
        if os.path.isfile(pf):
            if is_ignored_file(f):
                continue
            print(pf)
        else:
            if (f == 'physics' or f == 'robot_windows' or f == 'remote_controls'):
                list_folder(pf)
            else:
                sys.stderr.write("unknow plugin: " + pf + "\n")


def list_worlds(w):
    """List valid world files."""
    print(w + '/')
    for f in os.listdir(w):
        wf = w + '/' + f
        if omit_match(wf):
            continue
        if os.path.isfile(wf):
            if is_ignored_file(f):
                continue
            else:
                print(wf)
        else:
            if f in ['textures', 'meshes']:
                continue
            else:
                list_worlds(wf)


def proto_should_have_icon(f):
    """Check if this PROTO file should have an icon.

    Hidden and deprecated PROTO nodes doesn't need an icon.
    """
    file = open(f, 'r')
    row = file.readlines()
    for line in row:
        if re.match(r'^#[^\n]*tags[^\n]*:[^\n]*hidden', line) or re.match(r'^#[^\n]*tags[^\n]*:[^\n]*deprecated', line):
            return False
        if not line.startswith('#'):
            return True


def list_protos(p):
    """List valid protos files and subdirectories.

    Skip the icons folder and cache files.
    """
    print(p + '/')
    firstIcon = True
    for f in os.listdir(p):
        pf = p + '/' + f
        if omit_match(pf):
            continue
        if os.path.isfile(pf):
            if is_ignored_file(f) or (f[0] == '.' and f.endswith('.cache')):
                continue
            if pf.endswith('.proto'):
                print(pf)
                if proto_should_have_icon(pf):
                    if firstIcon:
                        print(p + '/icons/')
                        firstIcon = False
                    icon = p + '/icons/' + f.replace('.proto', '.png')
                    if not os.path.isfile(icon):
                        sys.stderr.write("missing icon: " + icon + "\n")
                    print(icon)
            # elif not pf.endswith(('.png', '.jpg', '.jpeg', '.hdr', '.obj')):
            else:
                print(pf)
        else:
            if f in ['icons', 'textures', 'meshes']:
                continue
            else:
                list_protos(pf)


def list_projects(p):
    """List files and directories located in the current path that need to be released."""
    print(p + '/')
    for f in os.listdir(p):
        pf = p + '/' + f
        if omit_match(pf):
            continue
        if os.path.isfile(pf):
            if is_ignored_file(f):
                continue
            print(pf)
        else:
            if f == 'controllers':
                list_controllers(pf)
            elif f == 'plugins':
                list_plugins(pf)
            elif f == 'protos':
                list_protos(pf)
            elif f == 'worlds':
                list_worlds(pf)
            elif f == 'libraries':
                list_folder(pf)
            elif pf in recurse_in_projects:
                print(pf + " [recurse]")
            else:
                list_projects(pf)  # recurse in subprojects


list_projects('projects')
