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

"""Test for duplicated PROTO names."""
import unittest
import os
import fnmatch


class TestProtoNames(unittest.TestCase):
    """Unit test of the PROTO names."""

    def setUp(self):
        """Get all the PROTO names."""
        self.protos = []
        self.protoPath = {}
        for directory in ['projects', 'resources']:
            for rootPath, dirNames, fileNames in os.walk(os.environ['WEBOTS_HOME'] + os.sep + directory):
                for fileName in fnmatch.filter(fileNames, '*.proto'):
                    self.protos.append(fileName)
                    if fileName not in self.protoPath:
                        self.protoPath[fileName] = []
                    self.protoPath[fileName].append(rootPath)

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


if __name__ == '__main__':
    unittest.main()
