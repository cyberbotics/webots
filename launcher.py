import os
import subprocess


WEBOTS_HOME = os.environ['WEBOTS_HOME']
os.environ['PATH'] = os.path.join(WEBOTS_HOME, 'lib', 'controller') + os.pathsep + os.environ['PATH']
command = 'webots --stream="monitorActivity"'
controllers = {
  "e-puck": r"projects\robots\gctronic\e-puck\controllers\e-puck\e-puck.exe"
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
        os.environ['WEBOTS_CONTROLLER_NAME'] = name
        os.environ['WEBOTS_STDOUT_REDIRECT'] = '1'
        controller = controllers[name] if name in controllers else ''
        print('starting ' + controller)
        if not controller:
            continue
        controller_path = os.path.join(WEBOTS_HOME, os.path.dirname(controller))
        os.chdir(controller_path)
        controller_process = subprocess.Popen([os.path.basename(controller)],
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
