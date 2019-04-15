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

"""Test PROTO icons."""
import unittest
import os
import fnmatch
import re

from PIL import Image

ignoredProtos = [
    "projects/robots/mobsya/thymio/controllers/thymio2_aseba/aseba/clients/studio/plugins/ThymioVPL/UsageProfile.proto",
]


class TestIcons(unittest.TestCase):
    """Unit test of the PROTO icons."""

    def setUp(self):
        """Get all the PROTO files to be tested."""
        # 1. Get all the PROTO files from projects
        protos = []
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
                    protos.append(proto)
        # 2. filter-out the hidden PROTO files
        self.visibleProtos = []
        for proto in protos:
            file = open(proto, 'r')
            row = file.readlines()
            for line in row:
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
            icon = protoDirectory + os.sep + 'icons' + os.sep + protoName + '.png'
            self.assertTrue(
                os.path.isfile(icon),
                msg='missing icon: "%s"' % proto
            )
            image = Image.open(icon)
            width, height = image.size
            self.assertTrue(
                width == 128 and height == 128,
                msg='wrong resolution (icons should be 128x128) for icon: "%s"' % icon
            )


if __name__ == '__main__':
    unittest.main()
