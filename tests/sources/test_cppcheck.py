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

"""Test quality of the source code using Cppcheck."""
import unittest
import os
import multiprocessing
import shutil
import sys


class TestCppCheck(unittest.TestCase):
    """Unit test for CppCheck errors."""

    def setUp(self):
        """Set up called before each test."""
        self.WEBOTS_HOME = os.path.normpath(os.environ['WEBOTS_HOME'])
        self.reportFilename = os.path.join(self.WEBOTS_HOME, 'tests', 'cppcheck_report.txt')
        self.extensions = ['c', 'h', 'cpp', 'hpp', 'cc', 'hh', 'c++', 'h++']
        if (sys.platform.startswith('linux')):
            self.platformOptions = ' -D__linux__'
        elif (sys.platform.startswith('win32')):
            self.platformOptions = ' -D_WIN32'
        else:
            self.platformOptions = ' -D__APPLE__'

        with open(os.path.join(self.WEBOTS_HOME, 'resources', 'version.txt'), 'r') as file:
            version = file.readlines()[0].strip()
            self.platformOptions += ' -DLIBCONTROLLER_VERSION=' + version

    def test_cppcheck_is_correctly_installed(self):
        """Test Cppcheck is correctly installed."""
        self.assertTrue(
            shutil.which('cppcheck') is not None,
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

    def add_source_files(self, sourceDirs, skippedDirs, skippedFiles=[]):
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
                            shouldSkip = False
                            for skipped in skippedDirs + skippedFiles:
                                if line.startswith(skipped):
                                    shouldSkip = True
                                    break
                            if not shouldSkip:
                                command += ' \"' + line + '\"'
                            continue
            for source in skippedFiles:
                command += ' --suppress=\"*:' + source + '\"'
        else:
            for source in skippedFiles:
                command += ' --suppress=\"*:' + source + '\"'
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
            'src/controller/c',
            'src/controller/cpp',
            'src/controller/launcher',
            'resources/projects'
        ]
        skippedDirs = [
            'src/webots/build',
            'src/webots/external',
            'resources/projects/libraries/qt_utils/build',
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
        skippedFiles = [
            'src/controller/c/sha1.c',
            'src/controller/c/sha1.h'
        ]
        if not sys.platform.startswith('win32'):
            skippedFiles.append('src/webots/core/WbWindowsRegistry.hpp')
        command = 'cppcheck --platform=native --enable=warning,style,performance,portability --inconclusive -q'
        command += self.platformOptions
        command += ' --library=qt -j %s' % str(multiprocessing.cpu_count())
        command += ' --inline-suppr --suppress=invalidPointerCast --suppress=useStlAlgorithm --suppress=uninitMemberVar'
        command += ' --suppress=noCopyConstructor --suppress=noOperatorEq --suppress=strdupCalled --suppress=unknownMacro'
        # command += ' --xml '  # Uncomment this line to get more information on the errors
        command += ' --output-file=\"' + self.reportFilename + '\"'
        for include in includeDirs:
            command += ' -I\"' + include + '\"'
        sources = self.add_source_files(sourceDirs, skippedDirs, skippedFiles)
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
            'projects/robots/robotis/darwin-op/libraries/python',
            'projects/robots/robotis/darwin-op/libraries/robotis-op2/robotis',
            'projects/robots/robotis/darwin-op/remote_control/libjpeg-turbo',
            'projects/vehicles/controllers/ros_automobile/include',
            'projects/robots/gctronic/e-puck/plugins/robot_windows/botstudio/build',
            'projects/robots/nex/plugins/robot_windows/fire_bird_6_window/build',
            'projects/vehicles/plugins/robot_windows/automobile_window/build'
        ]
        skippedFiles = [
            'projects/robots/robotis/darwin-op/plugins/remote_controls/robotis-op2_tcpip/stb_image.h',
            'projects/robots/epfl/lis/plugins/physics/blimp_physics/utils.h'
        ]
        command = 'cppcheck --platform=native --enable=warning,style,performance,portability --inconclusive -q'
        command += self.platformOptions
        command += ' --library=qt --inline-suppr --suppress=invalidPointerCast --suppress=useStlAlgorithm -UKROS_COMPILATION'
        command += ' --suppress=strdupCalled --suppress=ctuOneDefinitionRuleViolation --suppress=unknownMacro'
        # command += ' --xml'  # Uncomment this line to get more information on the errors
        command += ' --std=c++03 --output-file=\"' + self.reportFilename + '\"'
        sources = self.add_source_files(sourceDirs, skippedDirs, skippedFiles)
        if not sources:
            return
        command += sources
        self.run_cppcheck(command)


if __name__ == '__main__':
    unittest.main()
