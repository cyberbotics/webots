import os
import subprocess
import sys

if sys.platform != 'linux':
    sys.exit('This script runs only on Linux, not on ' + sys.platform)
subprocess.run(['xhost', '+local:root'])
port = 1234
with open('docker/.env', 'w+') as env_file:
    env_file.write('IMAGE=cyberbotics/webots:R2022b-1\n')
    env_file.write('PORT=' + str(port) + '\n')

command = 'docker-compose -f docker/docker-compose-webots.yml up --build --no-color'
controllers = {
  "e-puck": "/webots_project/controllers/e-puck/e-puck"
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
    if line.startswith('start:'):
        split = line.split(':')
        name = split[1]
        os.environ['WEBOTS_ROBOT_NAME'] = name
        os.environ['WEBOTS_SERVER'] = split[2]
        os.environ['WEBOTS_STDOUT_REDIRECT'] = '1'
        os.environ['WEBOTS_STDERR_REDIRECT'] = '1'
        controller = controllers[name] if name in controllers else ''
        print('starting ' + controller)
        if not controller:
            continue
    elif line:
        print('line: ' + line)
    if controller_process:
        while controller_process.poll() is None:
            line = controller_process.stdout.readline().rstrip()
            if line:
                print('controller: ' + line)
os.remove('docker/.env')
