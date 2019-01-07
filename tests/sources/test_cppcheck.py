# Copyright 1996-2018 Cyberbotics Ltd.
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
        """Set up the include and source directories."""
        self.WEBOTS_HOME = os.environ['WEBOTS_HOME']
        self.reportFilename = self.WEBOTS_HOME + '/tests/cppcheck_report.txt'

        self.cppcheck = 'cppcheck'
        if 'TRAVIS' in os.environ:
            self.cppcheck = self.WEBOTS_HOME + '/tests/sources/bin/cppcheck'

        self.includeDirs = [
            'include/controller/c',
            'include/ode',
            'include/qt/QtCore',
            'include/qt/QtGui',
            'include/qt/QtWidgets',
            'include/qt/QtPrintSupport',
            'include/qt/QtOpenGL',
            'include/qt/QtNetwork',
            'include/qt/QtWebEngineCore',
            'include/qt/QtWebEngineWidgets',
            'include/qt/QtWebChannel',
            'include/qt/QtWebSockets',
            'include/qt/QtXml',
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

        self.sourceDirs = [
            'src/webots',
            'src/wren',
            'src/lib/Controller',
            'resources/languages/cpp',
            'resources/projects'
        ]

        self.skippedDirs = [
            'src/webots/external',
            'include/opencv2'
        ]

        self.projectsSourceDirs = [
            'projects'
        ]

        self.projectsSkippedDirs = [
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

    def test_cppcheck_is_correctly_installed(self):
        """Test Cppcheck is correctly installed."""
        self.assertTrue(
            find_executable(self.cppcheck) is not None,
            msg='Cppcheck is not installed on this computer.'
        )

    def run_cppcheck(self, command):
        """Run Cppcheck command and check for errors."""
        if os.path.isfile(self.reportFilename):
            os.remove(self.reportFilename)

        if 'TRAVIS' not in os.environ:
            os.system(command)

        if os.path.isfile(self.reportFilename):
            reportFile = open(self.reportFilename, 'r')
            reportText = reportFile.read()
            self.assertTrue(
                len(reportText) == 0,
                msg='Cppcheck detected some errors:\n\n%s' % reportText
            )
            reportFile.close()
            os.remove(self.reportFilename)

    def test_sources_with_cppcheck(self):
        """Test Webots with Cppcheck."""
        command = self.cppcheck + ' --enable=warning,style,performance,portability --inconclusive --force -q'
        command += ' -j %s' % str(multiprocessing.cpu_count())
        command += ' --inline-suppr --suppress=invalidPointerCast --output-file=' + self.reportFilename
        for include in self.includeDirs:
            command += ' -I\"' + os.path.normpath(self.WEBOTS_HOME + '/' + include) + '\"'
        for source in self.skippedDirs:
            command += ' -i\"' + os.path.normpath(self.WEBOTS_HOME + '/' + source) + '\"'
        for source in self.sourceDirs:
            command += ' \"' + os.path.normpath(self.WEBOTS_HOME + '/' + source) + '\"'
        self.run_cppcheck(command)

    def test_projects_with_cppcheck(self):
        """Test projects with Cppcheck."""
        command = self.cppcheck + ' --enable=warning,style,performance,portability --inconclusive --force -q '
        command += '--inline-suppr --suppress=invalidPointerCast -UKROS_COMPILATION --std=c++03 --output-file=' + self.reportFilename
        for source in self.projectsSkippedDirs:
            command += ' -i\"' + os.path.normpath(self.WEBOTS_HOME + '/' + source) + '\"'
        for source in self.projectsSourceDirs:
            command += ' \"' + os.path.normpath(self.WEBOTS_HOME + '/' + source) + '\"'
        self.run_cppcheck(command)


if __name__ == '__main__':
    unittest.main()
