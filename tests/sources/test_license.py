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

"""Test that checks that all the source files have the Apache 2 license."""

import unittest
import datetime
import os
import fnmatch


APACHE2_LICENSE_C = """/*
 * Copyright 1996-20XX Cyberbotics Ltd.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */""".replace('20XX', str(datetime.datetime.now().year))

APACHE2_LICENSE_CPP = """// Copyright 1996-20XX Cyberbotics Ltd.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.""".replace('20XX', str(datetime.datetime.now().year))

APACHE2_LICENSE_PYTHON = """# Copyright 1996-20XX Cyberbotics Ltd.
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
# limitations under the License.""".replace('20XX', str(datetime.datetime.now().year))

PYTHON_OPTIONAL_HEADER = """#!/usr/bin/env python

"""


class TestLicense(unittest.TestCase):
    """Unit test for checking that all the source files have the Apache 2 license."""

    def setUp(self):
        """Get all the source files which require a license check."""
        directories = [
            'src/lib/Controller',
            'src/webots',
            'src/wren',
            'projects',
            'include/controller',
            'include/plugins',
            'resources/languages/cpp'
        ]

        skippedDirectoryPaths = [
            'src/webots/external',
            'projects/default/controllers/ros/include',
            'projects/default/resources/sumo',
            'projects/default/libraries/vehicle/java',
            'projects/default/libraries/vehicle/python',
            'projects/robots/epfl/lis/controllers/blimp',
            'projects/robots/epfl/lis/plugins/physics/blimp_physics',
            'projects/robots/gctronic/e-puck/transfer/library',
            'projects/robots/gctronic/e-puck/transfer/xc16',
            'projects/robots/mobsya/thymio/controllers/thymio2_aseba/aseba',
            'projects/robots/mobsya/thymio/libraries/dashel',
            'projects/robots/mobsya/thymio/libraries/dashel-src',
            'projects/robots/robotis/darwin-op/libraries/robotis-op2/robotis/Framework',
            'projects/robots/robotis/darwin-op/libraries/robotis-op2/robotis/Linux',
            'projects/robots/robotis/darwin-op/remote_control/libjpeg-turbo',
            'projects/robots/robotis/darwin-op/libraries/python',
            'projects/samples/robotbenchmark',
            'projects/vehicles/controllers/ros_automobile/include'
        ]

        skippedFilePaths = [
            'projects/robots/gctronic/e-puck/controllers/e-puck2_server/play_melody.c',
            'projects/robots/gctronic/e-puck/controllers/e-puck2_server/play_melody.h'
        ]

        skippedDirectories = [
            'build'
        ]

        extensions = ['*.c', '*.cpp', '*.h', '*.hpp', '*.py', '*.java', 'Makefile']

        self.sources = []
        for directory in directories:
            for rootPath, dirNames, fileNames in os.walk(os.environ['WEBOTS_HOME'] + os.sep + directory.replace('/', os.sep)):
                shouldContinue = False
                relativeRootPath = rootPath.replace(os.environ['WEBOTS_HOME'] + os.sep, '')
                for path in skippedDirectoryPaths:
                    if rootPath.startswith(os.environ['WEBOTS_HOME'] + os.sep + path.replace('/', os.sep)):
                        shouldContinue = True
                        break
                currentDirectories = rootPath.replace(os.environ['WEBOTS_HOME'], '').split(os.sep)
                for directory in skippedDirectories:
                    if directory in currentDirectories:
                        shouldContinue = True
                        break
                if fileNames == '__init__.py':
                    shouldContinue = True
                if shouldContinue:
                    continue
                for extension in extensions:
                    for fileName in fnmatch.filter(fileNames, extension):
                        if os.path.join(relativeRootPath, fileName) in skippedFilePaths:
                            continue
                        file = os.path.join(rootPath, fileName)
                        self.sources.append(file)

    def test_sources_have_license(self):
        """Test that sources have the license."""
        for source in self.sources:
            with open(source, 'r') as content_file:
                content = content_file.read()
                if source.endswith('.c') or source.endswith('.h'):
                    self.assertTrue(
                        content.startswith(APACHE2_LICENSE_C),
                        msg='Source file "%s" doesn\'t contain the correct Apache 2.0 License:\n%s' % (source, APACHE2_LICENSE_C)
                    )
                elif source.endswith('.cpp') or source.endswith('.hpp') or source.endswith('.java'):
                    self.assertTrue(
                        content.startswith(APACHE2_LICENSE_CPP),
                        msg='Source file "%s" doesn\'t contain the correct Apache 2.0 License:\n%s' % (source, APACHE2_LICENSE_CPP)
                    )
                elif source.endswith('.py') or source.endswith('Makefile'):
                    self.assertTrue(
                        content.startswith(APACHE2_LICENSE_PYTHON) or content.startswith(PYTHON_OPTIONAL_HEADER + APACHE2_LICENSE_PYTHON),
                        msg='Source file "%s" doesn\'t contain the correct Apache 2.0 License:\n%s' % (source, APACHE2_LICENSE_PYTHON)
                    )
                else:
                    self.assertTrue(
                        False,
                        msg='Unsupported file extension "%s".' % source
                    )


if __name__ == '__main__':
    unittest.main()
