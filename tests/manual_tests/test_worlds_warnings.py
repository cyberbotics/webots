#!/usr/bin/env python3

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

"""Test all the released worlds to make sure they don't generate warnings."""
import unittest
import os
import fnmatch
import sys
from threading import Timer
from subprocess import Popen, PIPE

if sys.version_info[0] != 3 or sys.version_info[1] < 7:
    sys.exit('This script requires Python version 3.7')


class TestWorldsWarnings(unittest.TestCase):
    """Unit test of the worlds."""

    def setUp(self):
        """Get all the worlds."""
        self.crashError = '(core dumped) "$webotsHome/bin/webots-bin" "$@"'
        self.skippedMessages = [
            'AL lib: (WW) alc_initconfig: Failed to initialize backend "pulse"',
            'ComposedShader is experimental.',
            # the ros controller of complete_test.wbt is started when loading the world because the robot-window is open
            'Failed to contact master at',
            self.crashError  # To remove once #6125 is fixed
        ]
        # Get all the worlds from projects
        self.worlds = []
        for directory in ['projects']:
            for rootPath, dirNames, fileNames in os.walk(os.environ['WEBOTS_HOME'] + os.sep + directory):
                for fileName in fnmatch.filter(fileNames, '*.wbt'):
                    world = os.path.join(rootPath, fileName)
                    self.worlds.append(world)
        self.webotsFullPath = None
        if sys.platform == 'win32':
            self.webotsFullPath = os.environ['WEBOTS_HOME'] + os.sep + 'msys64' + \
                os.sep + 'mingw64' + os.sep + 'bin' + os.sep + 'webots.exe'
        else:
            webotsBinary = 'webots'
            if 'WEBOTS_HOME' in os.environ:
                self.webotsFullPath = os.environ['WEBOTS_HOME'] + os.sep + webotsBinary
            else:
                self.webotsFullPath = '..' + os.sep + '..' + os.sep + webotsBinary
            if not os.path.isfile(self.webotsFullPath):
                print('Error: ' + webotsBinary + ' binary not found')
                sys.exit(1)
            self.webotsFullPath = os.path.normpath(self.webotsFullPath)

    def stop_webots(self):
        """Stop the Webots process."""
        self.process.terminate()

    def test_worlds_warnings(self):
        """Test all the 'projects' worlds for loading warnings."""
        problematicWorlds = []
        crashedWorlds = []
        for world in self.worlds:
            print('Testing: %s' % world)
            self.process = Popen([
                self.webotsFullPath,
                '--stdout',
                '--stderr',
                '--mode=pause',
                '--minimize',
                '--batch',
                world
            ], stdin=PIPE,
                stdout=PIPE, stderr=PIPE, text=True)
            t = Timer(20.0, self.stop_webots)
            t.start()
            output, error = self.process.communicate()
            if error and not any(message in str(error) for message in self.skippedMessages):
                problematicWorlds.append(world)
            if error and self.crashError in str(error):
                crashedWorlds.append(world)
        if crashedWorlds:
            print('\n\t'.join(['Impossible to test the following worlds because they crash when loading:'] + crashedWorlds))
        self.assertTrue(
            not problematicWorlds,
            msg='\n\t'.join(['The following worlds have unwanted warnings:'] + problematicWorlds)
        )


if __name__ == '__main__':
    unittest.main()
