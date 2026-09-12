#!/usr/bin/env python

# Copyright 1996-2024 Cyberbotics Ltd.
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

"""Test that all the required MATLAB functions are defined."""
import unittest
import glob
import os
import re
import subprocess
import sys

WEBOTS_HOME = os.path.normpath(os.environ['WEBOTS_HOME'])
sys.path.append(os.path.join(WEBOTS_HOME, 'src', 'controller', 'matlab'))
import mgenerate  # noqa: E402


class TestMatlabFunctions(unittest.TestCase):
    """Unit test for checking that all the required MATLAB functions are defined."""

    def setUp(self):
        if sys.version_info[0] >= 3:
            mgenerate.UPDATE = False
            mgenerate.main()
            """Get all the required functions."""
            skippedLines = [
                # Patterns
                '.*_H',  # Header Guards
                '.*_',  # Some comments use the ..._* pattern to refer to a group of functions ;
                        # grep will filter the *, but no function should end with _
                'wb_camera_image_get_.*',  # The MATLAB API exposes the image data as a multidimensional array,
                                           # so these functions are not needed
                'wb_(microphone|radio)_.*',  # Experimental Node Types
                'wb_remote_control_.*', 'wbr_.*',  # Remote Control Plugin
                'wb_robot_.*',  # Many robot functions are used internally by the C API (e.x. wb_robot_mutex_*)
                '.*_lookup_table_size',  # MATLAB lets you get the size of an array, so these functions are not needed
                'wbu_string_.*',  # String manipulation functions are not needed

                # Specific Functions

                # Non-Function Macros
                'WB_ALLOW_MIXING_C_AND_CPP_API',
                'WB_DEPRECATED',
                'WB_MATLAB_LOADLIBRARY',
                'WB_USING_C(PP)?_API',

                # These functions are used internally by the MATLAB API
                'wb_camera_recognition_get_object',
                'wb_lidar_get_point',
                'wb_mouse_get_state_pointer',
                'wb_radar_get_target',

                'wb_device_get_type',  # Deprecated since 8.0.0
                'wb_node_get_name',  # C API Only
            ]
            self.functions = []

            symbolSearch = subprocess.run([
                # Search for webots definitions
                'grep', '-Ehio', r'\bwb_\w+\b',
                # In the controller headers
                *glob.glob(os.path.join(WEBOTS_HOME, 'include', 'controller', 'c', 'webots', '*.h')),
                *glob.glob(os.path.join(WEBOTS_HOME, 'include', 'controller', 'c', 'webots', 'utils', '*.h'))
                ], capture_output=True, text=True)
            if symbolSearch.returncode != 0:
                self.fail(f'Failed to generate function list:\n{symbolSearch.stdout}\n{symbolSearch.stderr}')

            for line in symbolSearch.stdout.splitlines():
                if not any(re.match(f'^{skippedLine}$', line) for skippedLine in skippedLines) and line not in self.functions:
                    self.functions.append(line)

    @unittest.skipIf(sys.version_info[0] < 3, "not supported by Python 2.7")
    def test_matlab_function_exists(self):
        """Test that the function file exists."""
        expectedFiles = (os.path.join(WEBOTS_HOME, 'lib', 'controller', 'matlab', function + '.m')
                         for function in self.functions)
        missingFiles = [file for file in expectedFiles if not os.path.isfile(file)]

        self.assertTrue(
            not missingFiles,
            msg='Missing files: %s' % ', '.join(missingFiles)
        )


if __name__ == '__main__':
    unittest.main()
