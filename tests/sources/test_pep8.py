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

"""Test module of the PEP8 format in the tests."""
import unittest

import fnmatch
import pep8
import os

skippedDirectories = [
    'projects/robots/gctronic/e-puck/transfer',  #TODO: fix
    'projects/robots/softbank/nao/controllers/nao_demo_python',  #TODO: fix
    'projects/default/controllers/ros',  #TODO: fix
    'projects/default/controllers/sumo_supervisor',  #TODO: fix
    'projects/vehicles/worlds/sumo_interface_example_net',  #TODO
    'projects/samples/contests/rockin',  #TODO
    'projects/samples/robotbenchmark',  #TODO
    'projects/robots/mobsya/thymio/controllers/thymio2_aseba/aseba',
    'projects/robots/robotis/darwin-op/libraries/python',
    'projects/default/resources/sumo',
    'projects/languages/ros/controllers/ros_python/kinetic',
    'projects/languages/ros/controllers/ros_python/python'
]


class CustomReport(pep8.StandardReport):
    """Collect report, and overload the string operator."""

    results = []

    def get_file_results(self):
        """Overload this function to collect the report."""
        if self._deferred_print:
            self._deferred_print.sort()
            for line_number, offset, code, text, _ in self._deferred_print:
                self.results.append({
                    'path': self.filename,
                    'row': self.line_offset + line_number,
                    'col': offset + 1,
                    'code': code,
                    'text': text,
                })
        return self.file_errors

    def __str__(self):
        """Overload the string operator."""
        output = 'Report:\n'
        for result in self.results:
            output += '%s:%d:%d: %s: %s\n' % (
                result['path'],
                result['row'],
                result['col'],
                result['code'],
                result['text']
            )
        return output


class TestCodeFormat(unittest.TestCase):
    """Unit test of the PEP8 format in the tests."""

    def setUp(self):
        """Get all the world file."""
        self.files = []
        for rootPath, dirNames, fileNames in os.walk(os.environ['WEBOTS_HOME'] + os.sep + 'projects'):
            for fileName in fnmatch.filter(fileNames, '*.py'):
                shouldContinue = False
                for directory in skippedDirectories:
                    currentDirectories = rootPath.replace(os.environ['WEBOTS_HOME'], '')
                    if directory in currentDirectories:
                        shouldContinue = True
                        break
                if shouldContinue:
                    continue
                filePath = os.path.join(rootPath, fileName)
                self.files.append(filePath)

    def test_pep8_conformance(self):
        """Test that the tests are PEP8 compliant."""
        checker = pep8.StyleGuide(
            quiet=True,
            paths=self.files,
            reporter=CustomReport
        )
        checker.options.ignore = ('E501')  # E501: line too long (> 80 characters)
        report = checker.check_files()
        self.assertEqual(
            report.total_errors, 0,
            msg='PEP8 errors:\n%s' % (report)
        )


if __name__ == '__main__':
    unittest.main()
