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

"""Test PROTO icons."""
import unittest
import os
import fnmatch
import re

from PIL import Image

WEBOTS_HOME = os.path.normpath(os.environ['WEBOTS_HOME'])
ignoredProtos = [
    "projects/robots/mobsya/thymio/controllers/thymio2_aseba/aseba/aseba/clients/studio/plugins/ThymioVPL/UsageProfile.proto",
]
ignoredProtosFull = [os.path.join(WEBOTS_HOME, os.path.normpath(file)) for file in ignoredProtos]


class TestIcons(unittest.TestCase):
    """Unit test of the PROTO icons."""

    def setUp(self):
        """Get all the PROTO files to be tested."""
        # 1. Get all the PROTO files from projects
        protos = []
        for rootPath, dirNames, fileNames in os.walk(os.path.join(WEBOTS_HOME, 'projects')):
            for fileName in fnmatch.filter(fileNames, '*.proto'):
                proto = os.path.join(rootPath, fileName)
                if proto not in ignoredProtosFull:
                    protos.append(proto)
        # 2. filter-out the hidden PROTO files
        self.visibleProtos = []
        for proto in protos:
            with open(proto, 'r') as file:
                for line in file.readlines():
                    if re.match(r'^#.*tags.*:.*hidden', line) or re.match(r'^#.*tags.*:.*deprecated', line):
                        break
                    if not line.startswith('#'):
                        self.visibleProtos.append(proto)
                        break

    def test_proto_icons(self):
        """Test that the visible PROTO files have an icon."""
        for proto in self.visibleProtos:
            protoPath = os.path.splitext(proto)[0]
            protoName = os.path.basename(protoPath)
            protoDirectory = os.path.dirname(proto)
            icon = os.path.join(protoDirectory, 'icons', protoName + '.png')
            self.assertTrue(
                os.path.isfile(icon),
                msg='missing icon: "%s"' % proto
            )
            with Image.open(icon) as image:
                width, height = image.size
                self.assertTrue(
                    width == 128 and height == 128,
                    msg='wrong resolution (icons should be 128x128) for icon: "%s"' % icon
                )


if __name__ == '__main__':
    unittest.main()
