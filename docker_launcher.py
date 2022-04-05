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
subprocess.run(['xhost', '+local:root'])
port = 1234
worlds = ['e-puck.wbt', 'camera.wbt']
with open('docker/simulation/.env', 'w+') as env_file:
    env_file.write('IMAGE=cyberbotics/webots:R2022b-1\n')
    env_file.write(f'PORT={port}\n')
    env_file.write(f'WORLD=/webots_project/worlds/{worlds[1]}\n')

command = 'docker-compose -f docker/simulation/docker-compose-webots.yml up --build --no-color'
controllers = {
  "e-puck": "/webots_project/controllers/e-puck/e-puck",
  "MyBot": "/webots_project/controllers/camera/camera"
}
try:
    webots_process = subprocess.Popen(command.split(),
                                      stdout=subprocess.PIPE,
                                      stderr=subprocess.STDOUT,
                                      bufsize=1, universal_newlines=True)
except Exception:
    print(f"error: Unable to start docker-compose: {command}")
    quit()
print(f'docker-compose [{webots_process.pid}] started: "{command}"')
controller_process = None
while webots_process.poll() is None:
    line = webots_process.stdout.readline().rstrip()
    if line.startswith('webots_1  | '):  # output of the first docker container
        line = line[12:]
        if line.startswith('start:'):
            split = line.split(':')
            name = split[1]
            controller = controllers[name] if name in controllers else ''
            if not controller:
                print('controller "' + name + '" not found, skipping...')
                continue
            print('starting ' + controller)
            server = split[2]
            command = ('docker build -t controller '
                       '--build-arg WEBOTS_DEFAULT_IMAGE=cyberbotics/webots:R2022b-1 '
                       'docker/controller_1')
            run(command, True)
            command = (f'docker run -e WEBOTS_ROBOT_NAME={name} -e WEBOTS_SERVER={server} '
                       f'-v tmp-{port}:/tmp controller {controller}')
            subprocess.Popen(command.split())  # launch in the background
    elif line:
        print(line)  # docker-compose output
    if controller_process:
        while controller_process.poll() is None:
            line = controller_process.stdout.readline().rstrip()
            if line:
                print('controller: ' + line)
os.remove('docker/simulation/.env')
