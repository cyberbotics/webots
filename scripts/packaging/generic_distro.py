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

"""Generic functions to generate Webots package."""

from generate_projects_files import generate_projects_files, is_ignored_file, is_ignored_folder
import glob
import os
import re
import shutil
import stat
import sys
from abc import ABC, abstractmethod


def symlink_force(src, dst):
    try:
        if os.path.exists(dst):
            os.remove(dst)
        os.symlink(src, dst)
    except OSError as e:
        raise e


def remove_force(src):
    if not os.path.exists(src):
        return
    if os.path.isdir(src):
        shutil.rmtree(src)
    else:
        os.remove(src)


def print_error_message_and_exit(text):
    CONSOLE_RED_COLOR = "\033[22;31;1m"
    CONSOLE_DEFAULT_COLOR = "\033[22;30;0m"
    sys.exit(CONSOLE_RED_COLOR + text + CONSOLE_DEFAULT_COLOR)


def get_webots_version():
    # get package version
    try:
        with open(os.path.join(os.getenv('WEBOTS_HOME'), 'scripts', 'packaging', 'webots_version.txt'), 'r') as version_file:
            version = version_file.readline().strip()
            parts = version.split(' ')
            if len(parts) > 2:
                return parts[0] + '-rev' + parts[2]
            return parts[0]
    except IOError:
        print_error_message_and_exit("Could not open file: webots_version.txt.")


class WebotsPackage(ABC):
    def __init__(self, package_name):
        self.application_name = package_name
        self.application_name_lowercase_and_dashes = package_name.lower().replace(' ', '-')

        self.package_files = []
        self.package_folders = []

        # get WEBOTS_HOME
        try:
            self.webots_home = os.getenv('WEBOTS_HOME')
        except KeyError:
            sys.exit("WEBOTS_HOME not defined.")
        self.packaging_path = os.path.join(self.webots_home, 'scripts', 'packaging')

        # get distribution path
        self.distribution_path = ''
        custom_distribution_path = os.getenv('WEBOTS_DISTRIBUTION_PATH')
        if custom_distribution_path:
            if not os.path.isdir(custom_distribution_path):
                sys.exit(f"WEBOTS_DISTRIBUTION_PATH is set to a directory that doesn't exists: {custom_distribution_path}.")
            else:
                self.distribution_path = custom_distribution_path
        else:
            self.distribution_path = os.path.join(self.webots_home, 'distribution')
        if not os.path.isdir(self.distribution_path):
            sys.exit(f"Distribution path doesn't exists: {self.distribution_path}.")

        with open(os.path.join(os.getenv('WEBOTS_HOME'), 'scripts', 'packaging', 'webots_version.txt'), 'r') as version_file:
            self.full_version = version_file.readline().strip()
        self.package_version = get_webots_version()

        os.chdir(self.packaging_path)

    def create_webots_bundle(self, include_commit_file):
        # in an official distribution, the commit file should not be included and urls should reference the tag whereas on a
        # local distribution, the commit files must be included otherwise manufactured URLs would reference a tag which is not
        # guaranteed to exist
        if include_commit_file:
            self.add_files_from_string('resources/commit.txt')
        # populate self.package_folders and self.package_files
        print('listing core files')
        self.add_files(os.path.join(self.packaging_path, 'files_core.txt'))
        print('listing project files')
        self.add_files(generate_projects_files(os.path.join(self.webots_home, 'projects')))

    def test_file(self, filename):
        if os.path.isabs(filename) or filename.startswith('$'):
            return   # ignore absolute file names
        if filename.find('*') != -1:
            return  # ignore wildcard filenames
        local_file_path = os.path.join('..', '..', filename)
        if not os.access(local_file_path, os.F_OK):
            print_error_message_and_exit(f"Missing file: {filename}")

    def test_dir(self, dir_name):
        if dir_name == 'util':
            return  # this one is created and copied elsewhere
        local_dir_path = os.path.join('..', '..', dir_name)
        if not os.access(local_dir_path, os.F_OK):
            print_error_message_and_exit(f"Missing dir: {dir_name}")

    @abstractmethod
    def make_dir(self, directory):
        pass

    def copy_file(self, path):
        if is_ignored_file(os.path.basename(path)):
            print_error_message_and_exit(f"Trying to copy ignored file: {path}")
        self.test_file(path)

    def set_file_attribute(self, file_path, attribute):
        pass

    def set_execution_rights(self, file_path):
        mode = os.stat(file_path).st_mode
        os.chmod(file_path, mode | stat.S_IEXEC)

    @abstractmethod
    def compute_name_with_prefix_and_extension(self, basename, options):
        return ""

    def add_folder_recursively(self, folder_path):
        if is_ignored_folder(os.path.basename(folder_path)):
            return
        # add this folder and its content to self.package_folders and self.package_files
        self.package_folders.append(os.path.relpath(folder_path, self.webots_home))
        for subfile in os.listdir(folder_path):
            subfile_path = os.path.join(folder_path, subfile)
            if os.path.isdir(subfile_path):
                self.add_folder_recursively(subfile_path)
            else:
                self.add_file(subfile_path)

    def add_file(self, file_path):
        # add this file and self.package_files
        # if WBT file, then also add associated WBPROJ

        absolute_path = os.path.abspath(file_path)
        if not os.path.exists(absolute_path):
            print_error_message_and_exit(f"File doesn't exists \"{absolute_path}\". "
                                         "This can be caused by a broken symbolic link.")

        if not os.path.isfile(absolute_path) or is_ignored_file(os.path.basename(file_path)):
            return
        self.package_files.append(os.path.relpath(absolute_path, self.webots_home))

        basename = os.path.basename(file_path)
        dirname = os.path.dirname(absolute_path)
        if file_path.endswith('.wbt') and str(os.path.sep + 'worlds' + os.path.sep) in file_path:
            # copy the .*.wbproj hidden file
            world_project_file = os.path.join(dirname, '.' + re.sub(r'.wbt$', '.wbproj', basename))
            self.set_file_attribute(world_project_file, 'hidden')
            self.package_files.append(os.path.relpath(world_project_file, self.webots_home))

    def add_files_from_string(self, line):
        # add this file or folder to self.package_files and self.package_folders

        # strip line and remove comments
        line = re.sub(r'#.*$', '', line)
        line = line.strip()
        if not line:
            return
        # extract options
        match = re.search(r'\[(.*)\]$', line)
        if match:
            options = match.group(1).split(',')
            name = line[:match.span()[0]].strip()
        else:
            name = line
            options = []
        name = self.compute_name_with_prefix_and_extension(name, options)
        if not name:  # file not needed on this system
            return
        for expanded_path in glob.glob(os.path.join(self.webots_home, name)):
            abs_path = os.path.abspath(expanded_path)
            if 'recurse' in options:
                self.add_folder_recursively(abs_path)
            elif os.path.isdir(abs_path):
                abs_path = os.path.relpath(abs_path, self.webots_home)
                self.package_folders.append(abs_path)
            else:
                self.add_file(abs_path)

    def add_files(self, stream):
        # add these files and folders to self.package_files and self.package_folders
        if isinstance(stream, list):
            for file in stream:
                self.add_files_from_string(file)
            return

        with open(stream, 'r') as f:
            for line in f:
                self.add_files_from_string(line)
