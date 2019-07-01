#!/usr/bin/env python
"""This launcher simply start Webots."""

import optparse
import os
import sys
import subprocess

optParser = optparse.OptionParser()
optParser.add_option("--world", dest="world", default="", help="Path to the world to load")
optParser.add_option("--mode", dest="mode", default="realtime", help="Startup mod")
options, args = optParser.parse_args()

if 'WEBOTS_HOME' not in os.environ:
    sys.exit('WEBOTS_HOME environmental variable not defined.')
subprocess.call([os.path.join(os.environ['WEBOTS_HOME'], 'webots'), '--mode=' + options.mode, options.world])
