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
        WEBOTS_HOME = os.path.normpath(os.environ['WEBOTS_HOME'])
        self.worlds = []
        for rootPath, dirNames, fileNames in os.walk(os.path.join(WEBOTS_HOME, 'projects')):
            for fileName in fnmatch.filter(fileNames, '*.wbt'):
                world = os.path.join(rootPath, fileName)
                self.worlds.append(world)

        self.wbproj = []
        for rootPath, dirNames, fileNames in os.walk(os.path.join(WEBOTS_HOME, 'projects')):
            for fileName in fnmatch.filter(fileNames, '*.wbproj'):
                wbproj = os.path.join(rootPath, fileName)
                self.wbproj.append(wbproj)

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

    def test_wbproj(self):
        """Test that each world has an associated '.wbproj' file and vice-versa."""
        for world in self.worlds:
            worldFile = os.path.basename(world)
            projFile = '.' + worldFile.replace('wbt', 'wbproj')
            projFile = world.replace(worldFile, projFile)
            self.assertTrue(
                os.path.isfile(projFile),
                msg='Missing ".wproj" file: "%s"' % projFile
            )
        for wbproj in self.wbproj:
            wbprojFile = os.path.basename(wbproj)
            worldFile = wbprojFile.replace('wbproj', 'wbt')[1:]
            worldFile = wbproj.replace(wbprojFile, worldFile)
            self.assertTrue(
                os.path.isfile(worldFile),
                msg='.wbproj file not associated to any world: "%s"' % wbproj
            )


if __name__ == '__main__':
    unittest.main()
