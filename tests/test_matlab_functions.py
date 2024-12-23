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
from command import Command


class TestMatlabFunctions(unittest.TestCase):
    """Unit test for checking that all the required Matlab functions are defined."""

    def setUp(self):
        if sys.version_info[0] >= 3:
            mgenerate.UPDATE = False
            mgenerate.main()
            """Get all the required functions."""
            skippedLines = [
                ';', 'EXPORTS',  # Comments / Keywords
                'abstract_camera',  # Abstract Camera functions should be called through their variant functions
                                    # (e.g. wb_camera_get_*)
                'microphone', 'radio',  # Experimental Node Types
                'remote_control', 'wbr',  # Remote Control Plugin
                'init', 'cleanup',  # These are usually called automatically
                'robot',  # Many robot functions are used internally by the C API (e.x. wb_robot_mutex_*)
                'lookup_table_size',  # Matlab lets you get the size of an array, so these functions are not needed
                'no_mutex',  # "no_mutex" function variants should only be used internally
                'get_unique_id',  # Only used internally by the C API
                'wbu_string'  # String manipulation functions are not needed
            ]
            skippedFunctions = [
                # These functions are used internally by the Matlab API
                'wb_camera_recognition_get_object',
                'wb_lidar_get_point',
                'wb_mouse_get_state_pointer',
                'wb_radar_get_target',

                # These functions are part of the internal API
                'wb_file_get_extension',
                'wb_lidar_get_unique_id',
                'wb_range_finder_get_unique_id',

                'wb_device_get_type',  # Deprecated since 8.0.0
                'wb_node_get_name',  # C API Only

                # Not Yet Implemented
                'wb_supervisor_load_world',
                'wb_supervisor_save_world',
                'wbu_system_tmpdir',
                'wbu_system_webots_instance_path',
            ]
            self.functions = []

            filename = os.path.join(WEBOTS_HOME, 'src', 'controller', 'c', 'Controller.def')
            if not os.path.isfile(filename):
                if sys.platform == 'win32':
                    self.fail(f'Missing {filename}. Try rebuilding Webots.')
                else:
                    import shlex
                    binaryLib = os.path.join(WEBOTS_HOME, 'lib', 'controller', 'libController.so')
                    if not os.path.isfile(binaryLib):
                        self.fail(f'Missing {binaryLib}. Try rebuilding Webots.')
                    command = Command(
                        f"readelf -Ws {shlex.quote(binaryLib)} | awk '{{print $8}}' | sort > {shlex.quote(binaryLib)}")
                    command.run(shell=True)
                    if command.returncode != 0:
                        self.fail(f'Failed to generate {filename}.')

            with open(filename) as file:
                for line in file:
                    line = line.strip()
                    if (line.startswith('wb') and not any(skippedLine in line for skippedLine in skippedLines) and
                            not line[3:].isupper()):
                        function = line[:line.find(' ')]  # Remove any additional metadata
                        if function not in skippedFunctions:
                            self.functions.append(function)

    @unittest.skipIf(sys.version_info[0] < 3, "not supported by Python 2.7")
    @unittest.skipIf(sys.platform == 'darwin', "not supported on macOS")
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
