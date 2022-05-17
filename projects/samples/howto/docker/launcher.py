# Copyright 1996-2022 Cyberbotics Ltd.
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

import os
import subprocess
import sys


def run(command, sync):
    try:
        process = subprocess.Popen(command.split(),
                                   stdout=subprocess.PIPE,
                                   stderr=subprocess.STDOUT,
                                   bufsize=1, universal_newlines=True)
    except Exception:
        print(f"Error: Unable to start {command}")
        quit()
    print(f"Started {command}")
    if sync:
        while process.poll() is None:
            line = process.stdout.readline().rstrip()
            if line:
                print(line)
        return None
    else:
        return process


if sys.platform != 'linux':
    sys.exit('This script runs only on Linux, not on ' + sys.platform)

controllers = {
  "MyBot": "controllers/camera/camera"
}
port = 1234

subprocess.run(['xhost', '+local:root'])
with open('simulation/.env', 'w+') as env_file:
    env_file.write('IMAGE=cyberbotics/webots:R2022b-1\n')
    env_file.write(f'PORT={port}\n')
    env_file.write('ROBOT_NAME_1=MyBot\n')
    env_file.write('WORLD=/webots_project/worlds/camera.wbt\n')

command = 'docker-compose -f simulation/docker-compose-webots.yml up --build --no-color'
try:
    webots_process = subprocess.Popen(command.split(),
                                      stdout=subprocess.PIPE,
                                      stderr=subprocess.STDOUT,
                                      bufsize=1, universal_newlines=True)
except Exception:
    print(f"error: Unable to start docker-compose: {command}")
    quit()
command = ('docker build -t controller '
           '--build-arg WEBOTS_DEFAULT_IMAGE=cyberbotics/webots:R2022b-1 '
           '--build-arg MAKE=1 '
           'controllers/camera')
run(command, True)
controller_process = None
while webots_process.poll() is None:
    line = webots_process.stdout.readline().rstrip()
    if line.startswith('webots_1  | '):  # output of the first docker container
        line = line[12:]
        if line.startswith('ipc://'):
            name = line[line.rfind('/') + 1:]
            controller = controllers[name] if name in controllers else ''
            if not controller:
                print('controller "' + name + '" not found, skipping...')
                continue
            command = (f'docker run -e WEBOTS_CONTROLLER_URL=ipc://{port}/{name} '
                       f'-v tmp-{port}-{name}:/tmp/webots-{port}/ipc/{name} controller /webots_project/{controller}')
            subprocess.Popen(command.split())  # launch in the background
    elif line:
        print(line)  # docker-compose output
    if controller_process:
        while controller_process.poll() is None:
            line = controller_process.stdout.readline().rstrip()
            if line:
                print('controller: ' + line)
os.remove('simulation/.env')
