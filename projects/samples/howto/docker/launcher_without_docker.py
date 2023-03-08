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

import os
import subprocess
import sys


controllers = {
    "MyBot": "controllers/camera/camera"
}
port = 1235

WEBOTS_HOME = os.path.normpath(os.environ['WEBOTS_HOME'])
DOCKER_DEMO = os.path.join(WEBOTS_HOME, 'projects', 'samples', 'howto', 'docker')
path = os.path.join(WEBOTS_HOME, 'lib', 'controller')
if sys.platform == 'win32':
    os.environ['PATH'] = path + os.pathsep + os.environ['PATH']
elif sys.platform == 'darwin':
    if 'DYLD_LIBRARY_PATH' in os.environ:
        os.environ['DYLD_LIBRARY_PATH'] = path + os.pathsep + os.environ['DYLD_LIBRARY_PATH']
    else:
        os.environ['DYLD_LIBRARY_PATH'] = path
else:  # linux
    if 'LD_LIBRARY_PATH' in os.environ:
        os.environ['LD_LIBRARY_PATH'] = path + os.pathsep + os.environ['LD_LIBRARY_PATH']
    else:
        os.environ['LD_LIBRARY_PATH'] = path
command = 'webots' if sys.platform == 'win32' else os.path.join(WEBOTS_HOME, 'webots')
command += f' --port={port} --extern-urls '
command += os.path.join(DOCKER_DEMO, 'simulation', 'worlds', 'camera.wbt')
try:
    webots_process = subprocess.Popen(command.split(),
                                      stdout=subprocess.PIPE,
                                      stderr=subprocess.STDOUT,
                                      bufsize=1, universal_newlines=True)
except Exception:
    print(f"error: Unable to start Webots: {command}")
    quit()
controller_process = None
while webots_process.poll() is None:
    line = webots_process.stdout.readline().rstrip()
    if line.startswith('ipc://'):
        os.environ['WEBOTS_CONTROLLER_URL'] = line
        os.environ['WEBOTS_STDOUT_REDIRECT'] = '1'  # you may comment out these two
        os.environ['WEBOTS_STDERR_REDIRECT'] = '1'  # lines to disable redirections
        split = line.split('/')
        name = split[3]
        controller = controllers[name] if name in controllers else ''
        if not controller:
            continue
        controller_path = os.path.join(DOCKER_DEMO, os.path.dirname(controller))
        os.chdir(controller_path)
        controller_process = subprocess.Popen([os.path.join(DOCKER_DEMO, controller)],
                                              stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                              bufsize=1, universal_newlines=True
                                              )
    elif line:
        print(line)
    if controller_process:
        while controller_process.poll() is None:
            line = controller_process.stdout.readline().rstrip()
            if line:
                print('controller: ' + line)
