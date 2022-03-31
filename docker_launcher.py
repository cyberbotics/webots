import os
import subprocess
import sys

if sys.platform != 'linux':
    sys.exit('This script runs only on Linux, not on ' + sys.platform)
subprocess.run(['xhost', '+local:root'])
port = 1234
with open('.env', 'w') as env_file:
    env_file.write('IMAGE=cyberbotics/webots\n')
    env_file.write('PORT=' + str(port) + '\n')

command = f'docker-compose -f docker-compose-webots.yml up --build --no-color'
controllers = {
  "e-puck": "projects/robots/gctronic/e-puck/controllers/e-puck/e-puck"
}
try:
    webots_process = subprocess.Popen(command.split(),
                                      stdout=subprocess.PIPE,
                                      stderr=subprocess.STDOUT,
                                      bufsize=1, universal_newlines=True)
except Exception:
    print(f"error: Unable to start Webots: {command}")
    quit()
print(f'Webots [{webots_process.pid}] started: "{command}"')
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
        controller_path = os.path.join(WEBOTS_HOME, os.path.dirname(controller))
        os.chdir(controller_path)
        controller_process = subprocess.Popen([os.path.join(WEBOTS_HOME, controller)],
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
