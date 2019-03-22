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

"""Test suite for ROS."""

# important, you need to source 'webots_catkin_ws/devel/setup.bash' before
# to launch this script

import os
import sys
import time
from command import Command


class RosTestSuite(object):
    """Test suite for ROS."""

    def __init__(self):
        """Constructor."""
        self.roscoreCommand = None
        self.webotsCommand = None
        self.rosNodeCommand = None
        self.rosmasterStarted = False
        self.rosControllerStarted = False

    def getWebotsFullPath(self):
        """Get the path to Webots."""
        if sys.platform == 'win32':
            webotsBinary = 'webots.exe'
        else:
            webotsBinary = 'webots'
        if 'WEBOTS_HOME' in os.environ:
            webotsFullPath = os.environ['WEBOTS_HOME'] + os.sep + webotsBinary
        else:
            webotsFullPath = '..' + os.sep + '..' + os.sep + webotsBinary
        if not os.path.isfile(webotsFullPath):
            print('Error: %s binary not found.' % webotsBinary)
            sys.exit(1)
        webotsFullPath = os.path.normpath(webotsFullPath)
        if sys.platform == 'darwin':
            webotsFullPath += '.app/Contents/MacOS/webots'
        return webotsFullPath

    def waitForProcess(self, process, timeout):
        """Wait for a process."""
        startingTime = time.time()
        while time.time() - startingTime < timeout:
            output = os.popen("ps -e").read()
            if process in output:
                return True
            time.sleep(1)
        print(output)  # temporary debug print to check what are the running processes
        return False

    def launchRosCore(self, timeout):
        """Launch ROS main node."""
        self.roscoreCommand = Command('roscore')
        self.roscoreCommand.run(timeout=0, forceTermination=False)
        self.rosmasterStarted = self.waitForProcess('rosmaster', timeout)

    def launchWebots(self, timeout):
        """Launch Webots."""
        webotsFullPath = self.getWebotsFullPath()
        rosCompleteTestWorldPath = '..'
        if 'WEBOTS_HOME' in os.environ:
            rosCompleteTestWorldPath = os.environ['WEBOTS_HOME']
        rosCompleteTestWorldPath = \
            rosCompleteTestWorldPath + os.sep + 'projects' + os.sep + \
            'languages' + os.sep + 'ros' + os.sep + 'worlds' + os.sep + \
            'complete_test.wbt'
        self.webotsCommand = Command(
            webotsFullPath + ' ' + rosCompleteTestWorldPath + ' --mode=fast --stdout --stderr --batch')
        self.webotsCommand.run(timeout=0, forceTermination=False, silent=False, redirectionFile="webots_ros.log")
        # make sure ros controller is started before to return
        self.rosControllerStarted = self.waitForProcess('ros', timeout)

    def executeRosNode(self):
        """Execute the complete_test node."""
        self.rosNodeCommand = Command('rosrun webots_ros complete_test')
        self.rosNodeCommand.run(
            timeout=120, expectedString=None,
            silent=False, forceTermination=True
        )
        if self.rosNodeCommand.isTimeout:
            print('ERROR: ros complete test timeout.')
        if self.rosNodeCommand.returncode != 0:
            print('ERROR: ros node exit with error code: %d.' % self.rosNodeCommand.returncode)

    def closeProcesses(self):
        """"Close the started processes if any."""
        if self.roscoreCommand:
            self.roscoreCommand.stopMainProcess()
        if self.webotsCommand:
            self.webotsCommand.stopMainProcess()


test = RosTestSuite()
test.launchRosCore(timeout=10)
if test.rosmasterStarted:
    test.launchWebots(timeout=10)
    if test.rosControllerStarted:
        test.executeRosNode()
    else:
        print('ERROR: ros controller timeout.')
else:
    print('ERROR: rosmaster timeout.')
    # This is a temporary debug print to check if rosmaster is
    # started after some time or not started at all
    for x in range(10):
        test.waitForProcess('rosmaster', 10)
        if test.rosmasterStarted:
            print('ERROR: rosmaster started after: %d sec.' & ((x + 2) * 10))
            break

test.closeProcesses()
