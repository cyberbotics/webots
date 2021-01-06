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

"""Test header version."""
import unittest
import os
import fnmatch

ignoredProtos = [
    'projects/robots/mobsya/thymio/controllers/thymio2_aseba/aseba/clients/studio/plugins/ThymioVPL/UsageProfile.proto',
    'projects/samples/tutorials/protos/FourWheelsRobot.proto'
]

skippedDirectories = [
    'dependencies',
    'distribution',
    '.git'
]


class TestHeaderVersion(unittest.TestCase):
    """Unit test of the PROTO and world headers."""

    def setUp(self):
        """Get all the PROTO files to be tested."""
        # 1. Get Webots version (without revision)
        self.version = None
        with open(os.environ['WEBOTS_HOME'] + os.sep + 'resources' + os.sep + 'version.txt') as file:
            content = file.read()
            self.version = content.splitlines()[0].strip().split()[0]
        # 2. Get all the PROTO files
        self.files = []
        for rootPath, dirNames, fileNames in os.walk(os.environ['WEBOTS_HOME']):
            dirNames[:] = [d for d in dirNames if d not in skippedDirectories]
            for fileName in fnmatch.filter(fileNames, '*.proto'):
                proto = os.path.join(rootPath, fileName)
                shouldIgnore = False
                for ignoredProto in ignoredProtos:
                    path = os.environ['WEBOTS_HOME'] + os.sep + ignoredProto.replace('/', os.sep)
                    if proto == path:
                        shouldIgnore = True
                        break
                if not shouldIgnore:
                    self.files.append((proto, '#VRML_SIM %s utf8' % self.version))
        # 3. Get all the world files
        for rootPath, dirNames, fileNames in os.walk(os.environ['WEBOTS_HOME']):
            dirNames[:] = [d for d in dirNames if d not in skippedDirectories]
            for fileName in fnmatch.filter(fileNames, '*.wbt'):
                world = os.path.join(rootPath, fileName)
                self.files.append((world, '#VRML_SIM %s utf8' % self.version))
        # 4. Get all the .wbproj files
        for rootPath, dirNames, fileNames in os.walk(os.environ['WEBOTS_HOME']):
            dirNames[:] = [d for d in dirNames if d not in skippedDirectories]
            for fileName in fnmatch.filter(fileNames, '*.wbproj'):
                projFile = os.path.join(rootPath, fileName)
                self.files.append((projFile, 'Webots Project File version %s' % self.version))

    def test_header_version(self):
        """Test that the PROTO and world files have the correct header."""
        for currentFile in self.files:
            fileToTest = currentFile[0]
            with open(fileToTest) as file:
                content = file.read()
                if content == '':
                    continue
                line = content.splitlines()[0].strip()
                self.assertTrue(
                    line.startswith(currentFile[1]),
                    msg='Wrong header in file: "%s"' % fileToTest
                )


if __name__ == '__main__':
    unittest.main()
