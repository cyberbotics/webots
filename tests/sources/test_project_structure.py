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

"""Test project_structure."""
import unittest
import os
import fnmatch


class TestTextures(unittest.TestCase):
    """
    Unit test of the project structure.

    Currenlty on the worlds are test.
    """

    def setUp(self):
        """Get all the world file."""
        self.worlds = []
        for rootPath, dirNames, fileNames in os.walk(os.environ['WEBOTS_HOME'] + os.sep + 'projects'):
            for fileName in fnmatch.filter(fileNames, '*.wbt'):
                image = os.path.join(rootPath, fileName)
                self.worlds.append(image)

    def test_world_directories(self):
        """Test that the 'worlds' directory is correct."""
        for world in self.worlds:
            directory = os.path.dirname(world)
            # check that the world is in a 'worlds' directory
            self.assertTrue(
                os.path.basename(directory) == 'worlds',
                msg='This world is not in a "worlds" directory: "%s"' % world
            )
            # check that this 'worlds' directory is not nested
            directory = os.path.dirname(directory)
            while os.path.basename(directory) != 'projects':
                directory = os.path.dirname(directory)
                self.assertTrue(
                    'worlds' not in os.listdir(directory),
                    msg='This world is nested because an upper "worlds" directory exists: "%s"' % world
                )


if __name__ == '__main__':
    unittest.main()
