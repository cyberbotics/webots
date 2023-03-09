#!/usr/bin/env python

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

"""Generate and output the list of projects files to be included in the release package."""


import os
import sys
import fnmatch

if sys.platform == 'linux':
    import distro  # needed to retrieve the Ubuntu version

try:
    WEBOTS_HOME = os.getenv('WEBOTS_HOME')
except KeyError:
    sys.exit("WEBOTS_HOME not defined.")

# add all the files from the projects folder except:
# .gitignore
# OS generated files
# text editor backup files
# build folders


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


def is_ignored_folder(f):
    """Check if this file has to be ignored.

    Ignored folders includes:
    - build folders
    - com Java folders
    - __pycache__ Python folder
    """
    return f == 'build' or f == 'com' or f == '__pycache__'


class ProjectFilesGenerator:
    def __init__(self):
        if sys.platform == 'win32':
            self.dll_extension = '.dll'
            self.platform = 'windows'
        elif sys.platform == 'darwin':
            self.dll_extension = '.dylib'
            self.platform = 'mac'
        else:
            self.dll_extension = '.so'
            self.platform = 'linux'

        script_dir = os.path.dirname(os.path.realpath(__file__))
        with open(os.path.join(script_dir, "omit_in_projects.txt")) as f:
            self.omit_in_projects = f.read().splitlines()

        with open(os.path.join(script_dir, "omit_in_projects_" + self.platform + ".txt")) as f:
            self.omit_in_projects += f.read().splitlines()

        if 'SNAPCRAFT_PROJECT_NAME' in os.environ:
            with open(os.path.join(script_dir, "omit_in_projects_snap.txt")) as f:
                self.omit_in_projects += f.read().splitlines()

        self.omit_in_projects = [os.path.join(WEBOTS_HOME, line) for line in self.omit_in_projects]

        with open(os.path.join(script_dir, "recurse_in_projects.txt")) as f:
            self.recurse_in_projects = f.read().splitlines()
        self.recurse_in_projects = [os.path.join(WEBOTS_HOME, line) for line in self.recurse_in_projects]

        valid_environment = self.check_exist_in_projects(self.recurse_in_projects)

        with open(os.path.join(script_dir, "exist_in_projects.txt")) as f:
            self.exist_in_projects = f.read().splitlines()
        self.exist_in_projects_platform_path = "exist_in_projects"
        if sys.platform == 'linux':
            self.exist_in_projects_platform_path += "_" + self.platform + '_' + distro.version()
        self.exist_in_projects_platform_path += ".txt"
        with open(os.path.join(script_dir, self.exist_in_projects_platform_path)) as f:
            self.exist_in_projects += f.read().splitlines()
        self.exist_in_projects = [os.path.join(WEBOTS_HOME, line) for line in self.exist_in_projects]
        valid_environment = self.check_exist_in_projects(self.exist_in_projects) & valid_environment

        if not valid_environment:
            sys.exit(-1)

    def run(self, folder):
        return self.list_projects(folder)

    def check_exist_in_projects(self, files):
        """Check if the given files exist in the projects directory."""
        valid_environment = True
        for file in files:
            if not os.path.exists(os.path.join(WEBOTS_HOME, file)):
                sys.stderr.write("\x1B[31mFile or folder not found: " + file + "\x1b[0m\n")
                valid_environment = False
        return valid_environment

    def omit_match(self, f):
        """Check if current file or directory has to be omitted.

        Files and directories to be omitted are listed in the folling files:
        - omit_in_projects.txt
        - omit_in_projects_linux.txt
        - omit_in_projects_mac.txt
        - omit_in_projects_windows.txt
        - omit_in_projects_snap.txt
        """
        for fn in self.omit_in_projects:
            if fn.endswith('/') and (f + '/').startswith(fn):
                return True
            if fnmatch.fnmatch(f, fn):
                return True
        return False

    def list_folder(self, p):
        """List files to be released located in the current directory or in subdirectories.

        Skip .gitignore and build folders, mark EXE and DLL appropriately.
        """
        projects = [p + '/']
        for f in os.listdir(p):
            pf = p + '/' + f
            if self.omit_match(pf):
                continue
            if os.path.isfile(pf):
                if is_ignored_file(f):
                    continue
                if sys.platform == 'win32' and f.endswith('.exe'):
                    projects.append(p + '/' + os.path.splitext(f)[0] + ' [exe]')
                elif f.endswith(self.dll_extension):
                    lib = os.path.splitext(f)[0]
                    if sys.platform != 'win32' and lib.startswith('lib'):
                        lib = lib[3:]  # remove the 'lib' prefix on Linux and macOS
                    projects.append(p + '/' + lib + ' [dll]')
                elif sys.platform != 'win32' and os.access(pf, os.X_OK) and f.find('.') == -1:
                    projects.append(pf + ' [exe]')
                else:
                    projects.append(pf)
            else:
                if is_ignored_folder(f):
                    continue
                elif pf in self.recurse_in_projects:
                    projects.append(pf + " [recurse]")
                    continue
                projects += self.list_folder(pf)  # recurse
        return projects

    def list_controller(self, p):
        """List the absolute path of controllers files to be released located in the current directory."""
        projects = [p + '/']
        for f in os.listdir(p):
            pf = p + '/' + f
            if self.omit_match(pf):
                continue
            if os.path.isfile(pf):
                if is_ignored_file(f):
                    continue
                if sys.platform == 'win32' and f.endswith('.exe'):
                    projects.append(p + '/' + os.path.splitext(f)[0] + ' [exe]')
                elif sys.platform != 'win32' and os.access(pf, os.X_OK) and f.find('.') == -1:
                    projects.append(pf + ' [exe]')
                else:
                    projects.append(pf)
            else:
                if is_ignored_folder(f):
                    continue
                elif pf in self.recurse_in_projects:
                    projects.append(pf + " [recurse]")
                    continue
                projects += self.list_folder(pf)
        return projects

    def list_controllers(self, p):
        """List valid controllers files and directory."""
        projects = [p + '/']
        for f in os.listdir(p):
            pf = p + '/' + f
            if self.omit_match(pf):
                continue
            if os.path.isfile(pf):
                if is_ignored_file(f):
                    continue
                projects.append(pf)
            else:
                projects += self.list_controller(pf)
        return projects

    def list_plugins(self, p):
        """List valid plugins files and directories."""
        projects = [p + '/']
        for f in os.listdir(p):
            pf = p + '/' + f
            if self.omit_match(pf):
                continue
            if os.path.isfile(pf):
                if is_ignored_file(f):
                    continue
                projects.append(pf)
            elif f == 'physics' or f == 'robot_windows' or f == 'remote_controls':
                projects += self.list_folder(pf)
            else:
                sys.stderr.write("unknow plugin: " + pf + "\n")
        return projects

    def list_worlds(self, w):
        """List valid world files."""
        projects = [w + '/']
        for f in os.listdir(w):
            wf = w + '/' + f
            if self.omit_match(wf):
                continue
            if os.path.isfile(wf):
                if is_ignored_file(f):
                    continue
                else:
                    projects.append(wf)
            else:
                if f in ['textures', 'meshes']:
                    continue
                else:
                    projects += self.list_worlds(wf)
        return projects

    def list_projects(self, p):
        """List files and directories located in the current path that need to be released."""
        projects = [p + '/']
        for f in os.listdir(p):
            pf = p + '/' + f
            if self.omit_match(pf):
                continue
            if os.path.isfile(pf):
                if is_ignored_file(f):
                    continue
                projects.append(pf)
            else:
                if f == 'controllers':
                    projects += self.list_controllers(pf)
                elif f == 'plugins':
                    projects += self.list_plugins(pf)
                elif f == 'protos':
                    continue
                elif f == 'worlds':
                    projects += self.list_worlds(pf)
                elif f == 'libraries':
                    projects += self.list_folder(pf)
                elif pf in self.recurse_in_projects:
                    projects.append(pf + " [recurse]")
                else:
                    projects += self.list_projects(pf)  # recurse in subprojects
        return projects


def generate_projects_files(folder):
    generator = ProjectFilesGenerator()
    return generator.run(folder)


if __name__ == "__main__":
    for item in generate_projects_files(os.path.join(WEBOTS_HOME, 'projects')):
        print(item)
