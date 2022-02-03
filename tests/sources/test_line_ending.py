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

"""Test line ending"""
import unittest
import os
import fnmatch


class TestLineEnding(unittest.TestCase):
    """Unit test of the line endings"""

    def setUp(self):
        """Get all files to be tested"""

        self.files = []
        for root_path, _, file_names in os.walk(os.environ['WEBOTS_HOME']):
            for extension in ('*.proto', '*.wbt', '*.wbo', '*.forest'):
                for file_name in fnmatch.filter(file_names, extension):
                    self.files.append(os.path.join(root_path, file_name))

    def test_line_ending(self):
        """Test that all assets have the correct line-ending"""

        invalid_endings = [b'\r\n', b'\r']
        for file in self.files:
            with open(file, 'rb') as f:
                self.assertFalse(
                    any(x in f.read() for x in invalid_endings),
                    msg='Wrong line ending in file: "%s"' % file
                )


if __name__ == '__main__':
    unittest.main()
