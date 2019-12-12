"""Supervisor of the Robot Programming benchmark."""

from controller import Supervisor
import os
import sys

try:
    includePath = os.environ.get("WEBOTS_HOME") + "/projects/samples/robotbenchmark/include"
    includePath.replace('/', os.sep)
    sys.path.append(includePath)
    from robotbenchmark import robotbenchmarkRecord
except ImportError:
    sys.stderr.write("Warning: 'robotbenchmark' module not found.\n")
    sys.exit(0)

robot = Supervisor()

timestep = int(robot.getBasicTimeStep())

thymio = robot.getFromDef("THYMIO2")
translation = thymio.getField("translation")

tz = 0
running = True
while robot.step(timestep) != -1:
    t = translation.getSFVec3f()
    if running:
        percent = 1 - abs(0.25 - t[2]) / 0.25
        if percent < 0:
            percent = 0
        if t[2] > 0.01 and abs(tz - t[2]) < 0.0001:  # away from starting position and not moving any more
            message = "stop"
            running = False
        else:
            message = "percent"
        message += ":" + str(percent)
        robot.wwiSendText(message)
        tz = t[2]
    else:  # wait for record message
        message = robot.wwiReceiveText()
        if message:
            if message.startswith("record:"):
                record = robotbenchmarkRecord(message, "robot_programming", percent)
                robot.wwiSendText(record)
                break
            elif message == "exit":
                break

robot.simulationSetMode(Supervisor.SIMULATION_MODE_PAUSE)
