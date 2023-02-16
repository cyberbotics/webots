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

"""Test line ending"""
import unittest
import os

SKIPPED_DIRECTORIES = [
    'dependencies',
    '.git',
    'aseba',
    'xc16'
]

SKIPPED_FILES = [
    'cppcheck_report.txt'
]

EXTENSIONS_TO_CHECK = [
    '.proto', '.wbt', '.wbo', '.wbproj', '.forest', '.txt', '.md', '.c', '.h', '.cpp', '.hpp', '.py', '.java', '.m', '.bvh',
    '.motion', '.obj', '.dae'
]


class TestLineEnding(unittest.TestCase):
    """Unit test of the line endings"""

    def setUp(self):
        """Get all files to be tested"""

        self.files = []
        for root_path, dir_names, file_names in os.walk(os.path.normpath(os.environ['WEBOTS_HOME'])):
            dir_names[:] = [d for d in dir_names if d not in SKIPPED_DIRECTORIES]
            for file in file_names:
                if file.endswith(tuple(EXTENSIONS_TO_CHECK)) and file not in SKIPPED_FILES:
                    self.files.append(os.path.join(root_path, file))

    def test_line_ending(self):
        """Test that all files have the correct line-ending"""

        invalid_endings = [b'\r\n', b'\r']
        for file in self.files:
            with open(file, 'r+b') as f:
                self.assertFalse(
                    any(x in f.read() for x in invalid_endings),
                    msg='Wrong line ending in file: "%s"' % file
                )


if __name__ == '__main__':
    unittest.main()
