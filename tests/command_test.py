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

"""Test the command.py file."""

import sys
import os
from command import Command

f = open('slave.py', 'w')
f.write(
    "# File: slave.py\n"
    "import time, sys\n"
    "print('Start count')\n"
    "for i in range(1, 5):\n"
    "  print(str(i))\n\n"
    "  # important otherwise may trick the subprocess.stdout.readline function"
    "  sys.stdout.flush()\n"
    "  time.sleep(1)\n"
    "print('Stop count')\n"
)
f.close()

if sys.platform == 'win32':
    sys.stdout = os.fdopen(sys.stdout.fileno(), 'w', 0)

command = Command('python slave.py'.split())

command.run(silent=False)
print('command log (timeout = ' + str(command.timeout) + ', expectedString = ' +
      str(command.expectedString) + ', silent = ' + str(command.silent) + ')')
print('  command: ' + ' '.join(command.cmd))
print('  output: "' + command.output.replace('\n', ' - ') + '"')
print('  returncode: ' + str(command.returncode))
print('  expectedStringFound: ' + str(command.expectedStringFound))
print('  isTimeout: ' + str(command.isTimeout))

command.run(timeout=2, silent=False)
print('command log (timeout = ' + str(command.timeout) + ', expectedString = ' +
      str(command.expectedString) + ', silent = ' + str(command.silent) + ')')
print('  command: ' + ' '.join(command.cmd))
print('  output: "' + command.output.replace('\n', ' - ') + '"')
print('  returncode: ' + str(command.returncode))
print('  expectedStringFound: ' + str(command.expectedStringFound))
print('  isTimeout: ' + str(command.isTimeout))

command.run(timeout=6, silent=False)
print('command log (timeout = ' + str(command.timeout) + ', expectedString = ' +
      str(command.expectedString) + ', silent = ' + str(command.silent) + ')')
print('  command: ' + ' '.join(command.cmd))
print('  output: "' + command.output.replace('\n', ' - ') + '"')
print('  returncode: ' + str(command.returncode))
print('  expectedStringFound: ' + str(command.expectedStringFound))
print('  isTimeout: ' + str(command.isTimeout))

command.run(expectedString="3", silent=False)
print('command log (timeout = ' + str(command.timeout) + ', expectedString = ' +
      str(command.expectedString) + ', silent = ' + str(command.silent) + ')')
print('  command: ' + ' '.join(command.cmd))
print('  output: "' + command.output.replace('\n', ' - ') + '"')
print('  returncode: ' + str(command.returncode))
print('  expectedStringFound: ' + str(command.expectedStringFound))
print('  isTimeout: ' + str(command.isTimeout))
