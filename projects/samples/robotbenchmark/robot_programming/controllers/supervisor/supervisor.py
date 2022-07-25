"""Supervisor of the Robot Programming benchmark."""

from controller import Supervisor
import os
import sys

try:
    includePath = os.path.join(os.path.normpath(os.environ.get("WEBOTS_HOME")), 'projects', 'samples', 'robotbenchmark',
                               'include')
    sys.path.append(includePath)
    from robotbenchmark import robotbenchmarkRecord
except ImportError:
    sys.stderr.write("Warning: 'robotbenchmark' module not found.\n")
    sys.exit(0)

robot = Supervisor()

timestep = int(robot.getBasicTimeStep())

thymio = robot.getFromDef("THYMIO2")
translation = thymio.getField("translation")

tx = 0
running = True
while robot.step(timestep) != -1:
    t = translation.getSFVec3f()
    if running:
        percent = 1 - abs(0.25 + t[0]) / 0.25
        if percent < 0:
            percent = 0
        if t[0] < -0.01 and abs(t[0] - tx) < 0.0001:  # away from starting position and not moving any more
            message = "stop"
            running = False
        else:
            message = "percent"
        message += ":" + str(percent)
        robot.wwiSendText(message)
        tx = t[0]
    else:  # wait for record message
        message = robot.wwiReceiveText()
        while message:
            if message.startswith("record:"):
                record = robotbenchmarkRecord(message, "robot_programming", percent)
                robot.wwiSendText(record)
                break
            elif message == "exit":
                break
            message = robot.wwiReceiveText()

robot.simulationSetMode(Supervisor.SIMULATION_MODE_PAUSE)
