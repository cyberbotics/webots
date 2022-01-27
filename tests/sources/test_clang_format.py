#!/usr/bin/env python

# Copyright 1996-2021 Cyberbotics Ltd.
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

"""Test that the C, C++ and shader source code is compliant with ClangFormat."""
import unittest

import difflib
import os
import subprocess

from io import open

from distutils.spawn import find_executable


class TestClangFormat(unittest.TestCase):
    """Unit test for ClangFormat compliance."""

    def setUp(self):
        """Set up called before each test."""
        self.WEBOTS_HOME = os.environ['WEBOTS_HOME']

    def _runClangFormat(self, f):
        """Run clang format on 'f' file."""
        return subprocess.check_output(['clang-format', '-style=file', f])

    def test_clang_format_is_correctly_installed(self):
        """Test ClangFormat is correctly installed."""
        self.assertTrue(
            find_executable('clang-format') is not None,
            msg='ClangFormat is not installed on this computer.'
        )
        clangFormatConfigFile = self.WEBOTS_HOME + os.sep + '.clang-format'
        self.assertTrue(
            os.path.exists(clangFormatConfigFile),
            msg=clangFormatConfigFile + ' not found.'
        )

    def test_sources_are_clang_format_compliant(self):
        """Test that sources are ClangFormat compliant."""
        directories = [
            'include/controller',
            'projects',
            'resources/projects',
            'resources/wren/shaders',
            'tests',
            'include/wren',
            'src/controller/c',
            'src/controller/cpp',
            'src/license/sign',
            'src/webots',
            'src/wren'
        ]
        skippedPaths = [
            'projects/default/controllers/ros/include',
            'projects/robots/gctronic/e-puck/transfer',
            'projects/robots/mobsya/thymio/controllers/thymio2_aseba/aseba',
            'projects/robots/mobsya/thymio/libraries/dashel',
            'projects/robots/mobsya/thymio/libraries/dashel-src',
            'projects/robots/robotis/darwin-op/libraries/libssh',
            'projects/robots/robotis/darwin-op/libraries/libzip',
            'projects/robots/robotis/darwin-op/libraries/robotis-op2/robotis',
            'projects/robots/robotis/darwin-op/remote_control/libjpeg-turbo',
            'projects/vehicles/controllers/ros_automobile/include',
            'src/webots/external'
        ]
        skippedFiles = [
            'projects/robots/robotis/darwin-op/plugins/remote_controls/robotis-op2_tcpip/stb_image.h'
        ]
        skippedDirectories = [
            'build',
            'python',
            'java'
        ]
        extensions = ['c', 'h', 'cpp', 'hpp', 'cc', 'hh', 'c++', 'h++', 'vert', 'frag']
        modified_files = os.path.join(self.WEBOTS_HOME, 'tests', 'sources', 'modified_files.txt')
        sources = []
        if os.path.isfile(modified_files):
            with open(modified_files, 'r') as file:
                for line in file:
                    line = line.strip()
                    extension = os.path.splitext(line)[1][1:].lower()
                    if extension not in extensions:
                        continue
                    found = False
                    for directory in directories:
                        if line.startswith(directory):
                            found = True
                            break
                    if not found:
                        continue
                    found = False
                    for directory in skippedPaths + skippedFiles:
                        if line.startswith(directory):
                            found = True
                            break
                    if found:
                        continue
                    for directory in skippedDirectories:
                        currentDirectories = line.split(os.sep)
                        if directory in currentDirectories:
                            found = True
                    if found:
                        continue
                    sources.append(line.replace('/', os.sep))
        else:
            for directory in directories:
                path = self.WEBOTS_HOME + os.sep + directory.replace('/', os.sep)
                for rootPath, dirNames, fileNames in os.walk(path):
                    shouldContinue = False
                    for path in skippedPaths:
                        if rootPath.startswith(self.WEBOTS_HOME + os.sep + path.replace('/', os.sep)):
                            shouldContinue = True
                            break
                    for directory in skippedDirectories:
                        currentDirectories = rootPath.replace(self.WEBOTS_HOME, '').split(os.sep)
                        if directory in currentDirectories:
                            shouldContinue = True
                            break
                    if shouldContinue:
                        continue
                    for fileName in fileNames:
                        extension = os.path.splitext(fileName)[1][1:].lower()
                        if extension not in extensions:
                            continue
                        path = os.path.normpath(os.path.join(rootPath, fileName))
                        skipFile = False
                        for file in skippedFiles:
                            if os.path.normpath((self.WEBOTS_HOME + os.sep + file.replace('/', os.sep))) == path:
                                skipFile = True
                                break
                        if not skipFile:
                            sources.append(path)
        curdir = os.getcwd()
        os.chdir(self.WEBOTS_HOME)
        for source in sources:
            diff = ''
            with open(source, encoding='utf8') as file:
                try:
                    for line in difflib.context_diff(self._runClangFormat(source).decode('utf-8').splitlines(),
                                                     file.read().splitlines()):
                        diff += line + '\n'
                except UnicodeDecodeError:
                    self.assertTrue(False, msg='utf-8 decode problem in %s' % source)
                self.assertTrue(
                    len(diff) == 0,
                    msg='Source file "%s" is not compliant with ClangFormat:\n\nDIFF:%s' % (source, diff)
                )
        os.chdir(curdir)


if __name__ == '__main__':
    unittest.main()
