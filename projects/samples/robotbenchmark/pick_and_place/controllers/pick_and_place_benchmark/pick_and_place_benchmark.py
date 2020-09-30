"""Controller program to manage the benchmark."""
from controller import Supervisor
import math
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


def parseSecondsIntoReadableTime(seconds):
    """Generate a string based on seconds having the format "m:s:cs"."""
    minutes = seconds / 60
    absoluteMinutes = int(minutes)
    seconds = (minutes - absoluteMinutes) * 60
    absoluteSeconds = int(seconds)
    m = str(absoluteMinutes)
    if absoluteMinutes <= 9:
        m = '0' + m
    s = str(absoluteSeconds)
    if absoluteSeconds <= 9:
        s = '0' + s
    cs = str(int((seconds - absoluteSeconds) * 100))
    return m + ':' + s + ':' + cs


robot = Supervisor()

timestep = int(robot.getBasicTimeStep())

targetNode = robot.getFromDef("TARGET")
targetPosition = targetNode.getPosition()
targetPosition[1] = 0.0350

targetOrientation = targetNode.getOrientation()

boxNode = robot.getFromDef("PRODUCT")

time = 0
distance = 0
previousDistance = 0
boxPicked = False
notMovingStepCount = 0
while robot.step(timestep) != -1:
    position = boxNode.getPosition()
    distance = round(math.sqrt(math.pow(targetPosition[0] - position[0], 2) +
                               math.pow(targetPosition[2] - position[2], 2)), 4)
    time = robot.getTime()
    robot.wwiSendText("update: " + str(time) + " {0:.4f}".format(distance))

    if boxPicked and distance < 0.036 and position[1] < 0.156:
        if distance == previousDistance:
            notMovingStepCount += 1
            if notMovingStepCount > 10:
                break
        else:
            notMovingStepCount = 0
        previousDistance = distance
    elif not boxPicked and position[1] > 0.21:
        boxPicked = True

robot.wwiSendText("stop")

while robot.step(timestep) != -1:
    # wait for record message
    message = robot.wwiReceiveText()
    if message:
        if message.startswith("record:"):
            record = robotbenchmarkRecord(message, "pick_and_place", -time)
            robot.wwiSendText(record)
            break
        elif message == "exit":
            break

robot.simulationSetMode(Supervisor.SIMULATION_MODE_PAUSE)
