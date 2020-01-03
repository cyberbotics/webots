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

"""This launcher simply start Webots."""

import optparse
import os
import sys
import subprocess

optParser = optparse.OptionParser()
optParser.add_option("--world", dest="world", default="", help="Path to the world to load.")
optParser.add_option("--mode", dest="mode", default="realtime", help="Startup mode.")
optParser.add_option("--no-gui", dest="noGui", default="false", help="Start Webots with minimal GUI.")
options, args = optParser.parse_args()

if 'WEBOTS_HOME' not in os.environ:
    sys.exit('WEBOTS_HOME environment variable not defined.')
command = [os.path.join(os.environ['WEBOTS_HOME'], 'webots'), '--mode=' + options.mode, options.world]
if options.noGui == 'true':
    command.append('--stdout')
    command.append('--stderr')
    command.append('--batch')
    command.append('--no-sandbox')
    command.append('--minimize')

subprocess.call(command)
