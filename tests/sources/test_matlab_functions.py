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

"""Test that all the required Matlab functions are defined."""
import unittest
import os
import sys
WEBOTS_HOME = os.path.normpath(os.environ['WEBOTS_HOME'])
sys.path.append(os.path.join(WEBOTS_HOME, 'src', 'controller', 'matlab'))
import mgenerate  # noqa: E402


class TestMatlabFunctions(unittest.TestCase):
    """Unit test for checking that all the required Matlab functions are defined."""

    def setUp(self):
        if sys.version_info[0] >= 3:
            mgenerate.UPDATE = False
            mgenerate.main()
            """Get all the required functions."""
            skippedLines = [
                ';', 'EXPORTS', # Comments / Keywords
                'abstract_camera',  # Abstract Camera functions should be called through their variant functions (e.g. wb_camera_get_*)
                'microphone', 'radio', # Experimental Node Types
                'remote_control', 'wbr',  # Remote Control Plugin
                'init', 'cleanup',  # These are usually called automatically
                'robot',  # Many robot functions are used internally by the C API (e.x. wb_robot_mutex_*)
                'lookup_table_size',  # Matlab lets you get the size of an array, so these functions are not needed
                'no_mutex',  # "no_mutex" function variants should only be used internally
                'get_unique_id',  # Only used internally by the C API
                'wbu_string'  # String manipulation functions are not needed
            ]
            self.functions = []
            filename = os.path.join(WEBOTS_HOME, 'src', 'controller', 'c', 'Controller.def')
            self.assertTrue(
                os.path.isfile(filename),
                msg='Missing "%s" file.' % filename
            )
            with open(filename) as file:
                for line in file:
                    if line.strip().startswith('wb') and not any(skippedLine in line for skippedLine in skippedLines) and not line[3:].isupper():
                        line = line.strip()
                        self.functions.append(line[:line.find(' ')])  # Line is of the form "symbol_name @ address"

            # These functions are used internally by the Matlab API
            self.functions.remove('wb_camera_recognition_get_object')
            self.functions.remove('wb_lidar_get_point')
            self.functions.remove('wb_mouse_get_state_pointer')
            self.functions.remove('wb_radar_get_target')

            # These functions are part of the internal API
            self.functions.remove('wb_file_get_extension')
            self.functions.remove('wb_lidar_get_unique_id')
            self.functions.remove('wb_range_finder_get_unique_id')

            self.functions.remove('wb_device_get_type')  # Deprecated since 8.0.0
            self.functions.remove('wb_node_get_name') # C API Only

            # Not Yet Implemented
            self.functions.remove('wbu_system_tmpdir')
            self.functions.remove('wbu_system_tmpdir')
            self.functions.remove('wbu_system_webots_instance_path')

    @unittest.skipIf(sys.version_info[0] < 3, "not supported by Python 2.7")
    def test_matlab_function_exists(self):
        """Test that the function file exists."""
        for function in self.functions:
            filename = os.path.join(WEBOTS_HOME, 'lib', 'controller', 'matlab', function + '.m')
            self.assertTrue(
                os.path.isfile(filename),
                msg='Missing "%s" file.' % filename
            )


if __name__ == '__main__':
    unittest.main()
