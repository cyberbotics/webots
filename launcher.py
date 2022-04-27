import os
import subprocess
import sys


WEBOTS_HOME = os.environ['WEBOTS_HOME']
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
command += ' --stream="monitorActivity"'
controllers = {
  "e-puck": "docker/controller_1/controllers/e-puck/e-puck",
  "MyBot": "docker/controller_1/controllers/camera/camera"
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
