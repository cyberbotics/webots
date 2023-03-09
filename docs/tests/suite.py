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

"""Run all the tests."""

import unittest
import sys


if __name__ == "__main__":
    logfile = None
    loader = unittest.TestLoader()
    tests = loader.discover('.')
    testRunner = None
    if len(sys.argv) > 1:
        logfile = open(sys.argv[1], "w")
        testRunner = unittest.runner.TextTestRunner(logfile)
    else:
        testRunner = unittest.runner.TextTestRunner(verbosity=2, resultclass=unittest.TextTestResult)
    result = testRunner.run(tests)
    if logfile is not None:
        logfile.close()
    sys.exit(not result.wasSuccessful())
