"""Controller program to manage the benchmark.

It manages the perturbation and evaluates the peformance of the user
controller.
"""

from controller import Supervisor
import os
import random
import sys

try:
    includePath = os.environ.get("WEBOTS_HOME") + "/projects/samples/robotbenchmark/include"
    includePath.replace('/', os.sep)
    sys.path.append(includePath)
    from robotbenchmark import robotbenchmarkRecord
except ImportError:
    sys.stderr.write("Warning: 'robotbenchmark' module not found.\n")
    sys.exit(0)

# Get random generator seed value from 'controllerArgs' field
seed = 1
if len(sys.argv) > 1 and sys.argv[1].startswith('seed='):
    seed = int(sys.argv[1].split('=')[1])

robot = Supervisor()

timestep = int(robot.getBasicTimeStep())

jointParameters = robot.getFromDef("PENDULUM_PARAMETERS")
positionField = jointParameters.getField("position")

emitter = robot.getEmitter("emitter")
time = 0
force = 0
forceStep = 800
random.seed(seed)
run = True

while robot.step(timestep) != -1:
    if run:
        time = robot.getTime()
        robot.wwiSendText("time:%-24.3f" % time)
        robot.wwiSendText("force:%.2f" % force)

        # Detect status of inverted pendulum
        position = positionField.getSFFloat()
        if position < -1.58 or position > 1.58:
            # stop
            run = False
            robot.wwiSendText("stop")
        else:
            if forceStep <= 0:
                forceStep = 800 + random.randint(0, 400)
                force = force + 0.02
                toSend = "%.2lf %d" % (force, seed)
                if sys.version_info.major > 2:
                    toSend = bytes(toSend, "utf-8")
                emitter.send(toSend)
            else:
                forceStep = forceStep - 1
    else:
        # wait for record message
        message = robot.wwiReceiveText()
        if message:
            if message.startswith("record:"):
                record = robotbenchmarkRecord(message, "inverted_pendulum", time)
                robot.wwiSendText(record)
                break
            elif message == "exit":
                break

robot.simulationSetMode(Supervisor.SIMULATION_MODE_PAUSE)
