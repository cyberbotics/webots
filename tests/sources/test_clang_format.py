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

"""Test that the source code is compliant with ClangFormat."""
import unittest

import difflib
import os
import subprocess
import fnmatch

from distutils.spawn import find_executable


class TestClangFormat(unittest.TestCase):
    """Unit test for ClangFormat compliance."""

    def setUp(self):
        """Get all the C / C++ source files compliant with ClangFormat."""
        directories = [
            'include/controller',
            'projects',
            'resources/projects',
            'resources/languages/cpp',
            'resources/wren/shaders',
            'tests',
            'include/wren',
            'src/lib/Controller',
            'src/license/sign',
            'src/webots',
            'src/wren'
        ]

        skippedPathes = [
            'projects/default/controllers/ros/include',
            'projects/robots/gctronic/e-puck/transfer',
            'projects/robots/mobsya/thymio/controllers/thymio2_aseba/aseba',
            'projects/robots/mobsya/thymio/libraries/dashel',
            'projects/robots/mobsya/thymio/libraries/dashel-src',
            'projects/robots/robotis/darwin-op/libraries/robotis-op2/robotis',
            'projects/robots/robotis/darwin-op/remote_control/libjpeg-turbo',
            'projects/vehicles/controllers/ros_automobile/include',
            'src/webots/external'
        ]

        skippedDirectories = [
            'build',
            'python',
            'java'
        ]

        extensions = ['*.c', '*.cpp', '*.h', '*.hpp', '*.vert', '*.frag']

        self.sources = []
        for directory in directories:
            for rootPath, dirNames, fileNames in os.walk(os.environ['WEBOTS_HOME'] + os.sep + directory.replace('/', os.sep)):
                shouldContinue = False
                for path in skippedPathes:
                    if rootPath.startswith(os.environ['WEBOTS_HOME'] + os.sep + path.replace('/', os.sep)):
                        shouldContinue = True
                        break
                for directory in skippedDirectories:
                    currentDirectories = rootPath.replace(os.environ['WEBOTS_HOME'], '').split(os.sep)
                    if directory in currentDirectories:
                        shouldContinue = True
                        break
                if shouldContinue:
                    continue
                for extension in extensions:
                    for fileName in fnmatch.filter(fileNames, extension):
                        file = os.path.join(rootPath, fileName)
                        self.sources.append(file)

        # Get ClangFormat configuration file
        self.clangFormatConfigFile = os.environ['WEBOTS_HOME'] + os.sep + '.clang-format'

    def _runClangFormat(self, f):
        """Run clang format on 'f' file."""
        clangFormatCommand = "clang-format"
        if 'TRAVIS' in os.environ:
            clangFormatCommand = "clang-format-5.0"
        return subprocess.check_output([clangFormatCommand, "-style=file", f])

    def test_clang_format_is_correctly_installed(self):
        """Test ClangFormat is correctly installed."""
        self.assertTrue(
            find_executable('clang-format') is not None,
            msg='ClangFormat is not installed on this computer.'
        )
        self.assertTrue(
            os.path.exists(self.clangFormatConfigFile),
            msg=self.clangFormatConfigFile + ' not found.'
        )

    def test_sources_are_clang_format_compliant(self):
        """Test that sources are ClangFormat compliant."""
        for source in self.sources:
            diff = ''
            for line in difflib.context_diff(self._runClangFormat(source).splitlines(), open(source).read().splitlines()):
                diff += line + '\n'
            self.assertTrue(
                len(diff) == 0,
                msg='Source file "%s" is not compliant with ClangFormat:\n\nDIFF:%s' % (source, diff)
            )


if __name__ == '__main__':
    unittest.main()
