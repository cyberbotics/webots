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

"""Test for PROTO."""
import unittest
import os
import re
import fnmatch


class TestProtos(unittest.TestCase):
    """Unit test of the PROTO."""

    def setUp(self):
        """Get all the PROTO."""
        self.protos = []
        self.protoPath = {}
        self.protoFiles = []
        for directory in ['projects', 'resources']:
            for rootPath, dirNames, fileNames in os.walk(os.path.join(os.path.normpath(os.environ['WEBOTS_HOME']), directory)):
                for fileName in fnmatch.filter(fileNames, '*.proto'):
                    self.protos.append(fileName)
                    if fileName not in self.protoPath:
                        self.protoPath[fileName] = []
                    self.protoPath[fileName].append(rootPath)
                    self.protoFiles.append(os.path.join(rootPath, fileName))

    def test_proto_names(self):
        """Test that there are not 2 PROTO files with the same name."""
        self.warnedProtos = []
        for proto in self.protos:
            number = self.protos.count(proto)
            if number > 1 and proto not in self.warnedProtos:
                self.warnedProtos.append(proto)
                self.assertTrue(
                    number == 1,
                    msg='Duplicated "' + proto + '" PROTO name:\n\t' + '\n\t'.join(self.protoPath[proto])
                )

    def test_proto_field_comment(self):
        """Test that the PROTO field comment ends with a '.'."""
        for protoFile in self.protoFiles:
            with open(protoFile, 'r') as file:
                content = file.read()
                for i, match in enumerate(re.finditer(r'\[\n((.*\n)*)\]', content, re.MULTILINE)):  # get field header
                    for j, fieldMatche in enumerate(re.finditer(r'\s*.*ield.*#(.*)', match.group(1), re.MULTILINE)):
                        comment = fieldMatche.group(1).strip()
                        self.assertTrue(
                            comment.endswith('.'),
                            msg='Missing "." at end of field comment in "' + protoFile + '", comment: "' + comment + '".'
                        )


if __name__ == '__main__':
    unittest.main()
