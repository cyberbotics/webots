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

"""Test and save all the released worlds to make sure they don't generate warnings."""
import unittest
import os
import fnmatch
import sys
import platform
from subprocess import Popen, PIPE, TimeoutExpired

if sys.version_info[0] != 3 or sys.version_info[1] < 7:
    sys.exit('This script requires Python version 3.7')

if platform.system() == 'Darwin':
    CACHE_DIR = os.path.join(os.path.expanduser('~'), 'Library', 'Caches', 'Cyberbotics', 'Webots', 'assets')
elif platform.system() == 'Linux':
    CACHE_DIR = os.path.join(os.path.expanduser('~'), '.cache', 'Cyberbotics', 'Webots', 'assets')
elif platform.system() == 'Windows':
    CACHE_DIR = os.path.join(os.path.expanduser('~'), 'AppData', 'Local', 'Cyberbotics', 'Webots', 'cache', 'assets')


class TestWorldsWarnings(unittest.TestCase):
    """Unit test of the worlds."""

    def setUp(self):
        """Get all the worlds."""
        if 'WEBOTS_HOME' in os.environ:
            WEBOTS_HOME = os.path.normpath(os.environ['WEBOTS_HOME'])
        else:
            WEBOTS_HOME = ''
        self.crashError = '(core dumped) "$webotsHome/bin/webots-bin" "$@"'
        self.skippedMessages = [
            'AL lib: (WW) alc_initconfig: Failed to initialize backend "pulse"',
            # the ros controller of complete_test.wbt is started when loading the world because the robot-window is open
            'Failed to contact master at',
            'Cannot initialize the sound engine',
            self.crashError  # To remove once #6125 is fixed
        ]
        # Set empty.wbt as the first world (to trigger the 'System below the minimal requirements' message)
        self.worlds = [os.path.join(WEBOTS_HOME, 'resources', 'projects', 'worlds', 'empty.wbt')]
        # Get all the worlds from projects
        for directory in ['projects']:
            for rootPath, dirNames, fileNames in os.walk(os.path.join(WEBOTS_HOME, directory)):
                for fileName in fnmatch.filter(fileNames, '*.wbt'):
                    world = os.path.join(rootPath, fileName)
                    self.worlds.append(world)
        self.webotsFullPath = None
        if sys.platform == 'win32':
            self.webotsFullPath = os.path.join(WEBOTS_HOME, 'msys64', 'mingw64', 'bin', 'webots.exe')
        else:
            webotsBinary = 'webots'
            if 'WEBOTS_HOME' in os.environ:
                self.webotsFullPath = os.path.join(WEBOTS_HOME, webotsBinary)
            else:
                self.webotsFullPath = os.path.join('..', webotsBinary)
            if not os.path.isfile(self.webotsFullPath):
                print('Error: ' + webotsBinary + ' binary not found')
                sys.exit(1)
            self.webotsFullPath = os.path.normpath(self.webotsFullPath)

    def test_worlds_warnings_and_cache(self):
        """Test all the 'projects' worlds for loading warnings and if they reference un-cached assets."""
        problematicWorlds = []
        crashedWorlds = []
        worldsWithNonCachedAssets = []
        cacheSizeBefore = len(os.listdir(CACHE_DIR))
        for i in range(len(self.worlds)):
            print('Testing: %d/%d: %s' % (i + 1, len(self.worlds), self.worlds[i]))
            self.process = Popen([
                self.webotsFullPath,
                self.worlds[i],
                '--stdout',
                '--stderr',
                '--minimize',
                '--batch',
                '--mode=pause',
                '--update-world'
            ], stdin=PIPE, stdout=PIPE, stderr=PIPE, text=True)
            try:
                output, errors = self.process.communicate()
            except TimeoutExpired:
                self.process.kill()
                output, errors = self.process.communicate()

            # First world is empty.wbt, used to trigger the warning about system requirements not being met
            if i == 0:
                continue
            if errors and not all((any(message in error for message in self.skippedMessages) for error in errors.splitlines())):
                problematicWorlds.append(self.worlds[i])
            if errors and self.crashError in str(errors):
                crashedWorlds.append(self.worlds[i])

            # url.wbt and camera.wbt are exceptional as they contain non-cached assets on purpose, adapt cache size accordingly
            worldName = os.path.basename(self.worlds[i])
            if worldName == 'url.wbt':
                cacheSizeBefore += 22
            if worldName == 'camera.wbt':
                cacheSizeBefore += 26

            cacheSizeAfter = len(os.listdir(CACHE_DIR))
            if cacheSizeAfter - cacheSizeBefore > 0:
                worldsWithNonCachedAssets.append(self.worlds[i])
                cacheSizeBefore = cacheSizeAfter  # update the counter so it doesn't trigger again for the same reason

        if crashedWorlds:
            print('\n\t'.join(['Impossible to test the following worlds because they crash when loading:'] + crashedWorlds))
        self.assertTrue(
            not problematicWorlds,
            msg='\n\t'.join(['The following worlds have unwanted warnings:'] + problematicWorlds)
        )

        if worldsWithNonCachedAssets:
            print('\nThe following worlds reference non-cached assets:')
            for world in worldsWithNonCachedAssets:
                print('- ' + world)

        if crashedWorlds or problematicWorlds or worldsWithNonCachedAssets:
            self.fail('Problematic worlds have been found.')


if __name__ == '__main__':
    unittest.main()
