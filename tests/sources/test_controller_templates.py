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

"""Regression tests for controller wizard templates."""

import os
import unittest


class TestControllerTemplates(unittest.TestCase):
    """Check that new controller templates follow the world basic time step."""

    def setUp(self):
        webots_home = os.path.normpath(os.environ['WEBOTS_HOME'])
        self.templates_dir = os.path.join(webots_home, 'resources', 'templates', 'controllers')

    def _read_template(self, filename):
        with open(os.path.join(self.templates_dir, filename), encoding='utf-8') as file:
            return file.read()

    def test_c_template_uses_world_basic_time_step(self):
        content = self._read_template('template.c')
        self.assertIn('wb_robot_get_basic_time_step()', content)
        self.assertNotIn('#define TIME_STEP 64', content)

    def test_matlab_template_uses_world_basic_time_step(self):
        content = self._read_template('template.m')
        self.assertIn('TIME_STEP = wb_robot_get_basic_time_step();', content)
        self.assertNotIn('TIME_STEP = 64;', content)


if __name__ == '__main__':
    unittest.main()
