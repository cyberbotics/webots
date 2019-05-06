#!/usr/bin/env python

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

"""Test header version."""
import unittest
import os
import fnmatch

ignoredProtos = [
    "projects/robots/mobsya/thymio/controllers/thymio2_aseba/aseba/clients/studio/plugins/ThymioVPL/UsageProfile.proto",
    "projects/samples/tutorials/protos/FourWheelsRobot.proto"
]


class TestIcons(unittest.TestCase):
    """Unit test of the PROTO icons."""

    def setUp(self):
        """Get all the PROTO files to be tested."""
        # 1. Get Webots version (without revision)
        self.version = None
        with open(os.environ['WEBOTS_HOME'] + os.sep + 'resources' + os.sep + 'version.txt') as file:
            content = file.read()
            self.version = content.splitlines()[0].strip().split()[0]
        # 2. Get all the PROTO files from projects
        self.files = []
        for rootPath, dirNames, fileNames in os.walk(os.environ['WEBOTS_HOME'] + os.sep + 'projects'):
            for fileName in fnmatch.filter(fileNames, '*.proto'):
                proto = os.path.join(rootPath, fileName)
                shouldIgnore = False
                for ignoredProto in ignoredProtos:
                    path = os.environ['WEBOTS_HOME'] + os.sep + ignoredProto.replace('/', os.sep)
                    if proto == path:
                        shouldIgnore = True
                        break
                if not shouldIgnore:
                    self.files.append(proto)
        # 3. Get all the world files from projects
        for rootPath, dirNames, fileNames in os.walk(os.environ['WEBOTS_HOME'] + os.sep + 'projects'):
            for fileName in fnmatch.filter(fileNames, '*.wbt'):
                world = os.path.join(rootPath, fileName)
                self.files.append(world)

    def test_header_version(self):
        """Test that the PROTO and world files have the correct header."""
        for fileToTest in self.files:
            with open(fileToTest) as file:
                content = file.read()
                line = content.splitlines()[0].strip()
                self.assertTrue(
                    line == '#VRML_SIM %s utf8' % self.version,
                    msg='Wrong header in file: "%s"' % fileToTest
                )


if __name__ == '__main__':
    unittest.main()
