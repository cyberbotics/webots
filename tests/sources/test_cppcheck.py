#!/usr/bin/env python

# Copyright 1996-2020 Cyberbotics Ltd.
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

"""Test quality of the source code using Cppcheck."""
import unittest
import os
import multiprocessing

from distutils.spawn import find_executable


class TestCppCheck(unittest.TestCase):
    """Unit test for CppCheck errors."""

    def setUp(self):
        """Set up called before each test."""
        self.WEBOTS_HOME = os.environ['WEBOTS_HOME']
        self.reportFilename = os.path.join(self.WEBOTS_HOME, 'tests', 'cppcheck_report.txt')
        if 'TRAVIS' in os.environ and 'TRAVIS_OS_NAME' in os.environ and os.environ['TRAVIS_OS_NAME'] == 'linux':
            self.cppcheck = self.WEBOTS_HOME + '/tests/sources/bin/cppcheck'
        else:
            self.cppcheck = 'cppcheck'
        self.extensions = ['c', 'h', 'cpp', 'hpp', 'cc', 'hh', 'c++', 'h++']

    def test_cppcheck_is_correctly_installed(self):
        """Test Cppcheck is correctly installed."""
        self.assertTrue(
            find_executable(self.cppcheck) is not None,
            msg='Cppcheck is not installed on this computer.'
        )

    def run_cppcheck(self, command):
        """Run Cppcheck command and check for errors."""
        curdir = os.getcwd()
        os.chdir(self.WEBOTS_HOME)
        if os.path.isfile(self.reportFilename):
            os.remove(self.reportFilename)
        os.system(command)  # warning: on Windows, the length of command is limited to 8192 characters
        if os.path.isfile(self.reportFilename):
            with open(self.reportFilename, 'r') as reportFile:
                reportText = reportFile.read()
            self.assertTrue(
                not reportText,
                msg='Cppcheck detected some errors:\n\n%s' % reportText
            )
            os.remove(self.reportFilename)
        os.chdir(curdir)

    def add_source_files(self, sourceDirs, skippedDirs):
        command = ''
        modified_files = os.path.join(self.WEBOTS_HOME, 'tests', 'sources', 'modified_files.txt')
        if os.path.isfile(modified_files):
            with open(modified_files, 'r') as file:
                for line in file:
                    line = line.strip()
                    extension = os.path.splitext(line)[1][1:].lower()
                    if extension not in self.extensions:
                        continue
                    for sourceDir in sourceDirs:
                        if line.startswith(sourceDir):
                            for skippedDir in skippedDirs:
                                if not line.startswith(skippedDir):
                                    command += ' \"' + line + '\"'
                                    break
                            continue
        else:
            for source in skippedDirs:
                command += ' -i\"' + source + '\"'
            for source in sourceDirs:
                command += ' \"' + source + '\"'
        return command

    def test_sources_with_cppcheck(self):
        """Test Webots with Cppcheck."""
        sourceDirs = [
            'src/webots',
            'src/wren',
            'src/lib/Controller',
            'resources/languages/cpp',
            'resources/projects'
        ]
        skippedDirs = [
            'src/webots/external',
            'include/opencv2',
            'include/qt'
        ]
        includeDirs = [
            'include/controller/c',
            'include/wren',
            'include/glad',
            'src/webots/app',
            'src/webots/control',
            'src/webots/core',
            'src/webots/editor',
            'src/webots/engine',
            'src/webots/external',
            'src/webots/gui',
            'src/webots/license',
            'src/webots/maths',
            'src/webots/nodes',
            'src/webots/ode',
            'src/webots/plugins',
            'src/webots/scene_tree',
            'src/webots/sound',
            'src/webots/user_commands',
            'src/webots/util',
            'src/webots/vrml',
            'src/webots/widgets',
            'src/webots/wren'
        ]
        command = self.cppcheck + ' --enable=warning,style,performance,portability --inconclusive --force -q'
        command += ' -j %s' % str(multiprocessing.cpu_count())
        command += ' --inline-suppr --suppress=invalidPointerCast --suppress=useStlAlgorithm --suppress=uninitMemberVar '
        command += ' --suppress=noCopyConstructor  --suppress=noOperatorEq'
        # command += ' --xml '  # Uncomment this line to get more information on the errors
        command += ' --output-file=\"' + self.reportFilename + '\"'
        for include in includeDirs:
            command += ' -I\"' + include + '\"'
        sources = self.add_source_files(sourceDirs, skippedDirs)
        if not sources:
            return
        command += sources
        self.run_cppcheck(command)

    def test_projects_with_cppcheck(self):
        """Test projects with Cppcheck."""
        sourceDirs = [
            'projects/default',
            'projects/devices',
            'projects/humans',
            'projects/languages',
            'projects/objects',
            'projects/robots',
            'projects/samples',
            'projects/vehicles'
        ]
        skippedDirs = [
            'projects/default/controllers/ros/include',
            'projects/default/libraries/vehicle/java',
            'projects/default/libraries/vehicle/python',
            'projects/robots/gctronic/e-puck/transfer',
            'projects/robots/mobsya/thymio/controllers/thymio2_aseba/aseba',
            'projects/robots/mobsya/thymio/libraries/dashel',
            'projects/robots/mobsya/thymio/libraries/dashel-src',
            'projects/robots/robotis/darwin-op/libraries/python',
            'projects/robots/robotis/darwin-op/libraries/robotis-op2/robotis',
            'projects/robots/robotis/darwin-op/remote_control/libjpeg-turbo',
            'projects/vehicles/controllers/ros_automobile/include'
        ]
        command = self.cppcheck + ' --enable=warning,style,performance,portability --inconclusive --force -q '
        command += '--inline-suppr --suppress=invalidPointerCast --suppress=useStlAlgorithm -UKROS_COMPILATION '
        # command += '--xml '  # Uncomment this line to get more information on the errors
        command += '--std=c++03 --output-file=\"' + self.reportFilename + '\"'
        sources = self.add_source_files(sourceDirs, skippedDirs)
        if not sources:
            return
        command += sources
        self.run_cppcheck(command)


if __name__ == '__main__':
    unittest.main()
