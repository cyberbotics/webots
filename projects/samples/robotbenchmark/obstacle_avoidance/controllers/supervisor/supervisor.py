"""Supervisor of the Obstacle Avoidance benchmark."""

from controller import Supervisor
import os
import sys
import random
import math

try:
    sys.path.append(os.path.join(os.path.normpath(os.environ.get("WEBOTS_HOME")), 'projects', 'samples', 'robotbenchmark',
                                 'include'))
    from robotbenchmark import robotbenchmarkRecord
except ImportError:
    sys.stderr.write("Warning: 'robotbenchmark' module not found.\n")
    sys.exit(0)


def normalize(vector):
    length = math.sqrt(vector[0] * vector[0] + vector[1] * vector[1] + vector[2] * vector[2])
    if length == 0:
        return [0, 0, 1]
    else:
        return [vector[0] / length, vector[1] / length, vector[2] / length]


def randomlyPlaceObject(node):
    translationField = node.getField("translation")
    rotationField = node.getField("rotation")

    newPosition = [random.random() * 1.6 - 0.5, random.random() * 2 - 1, random.random() + 0.5]
    newRotation = normalize([random.random(), random.random(), random.random()]) + [random.random() * math.pi * 2]

    translationField.setSFVec3f(newPosition)
    rotationField.setSFRotation(newRotation)


random.seed()
robot = Supervisor()
thymio2 = robot.getFromDef("THYMIO2")

timestep = int(robot.getBasicTimeStep())

cubeObstaclesGroup = robot.getFromDef("CUBE_OBSTACLES")
cubeObstaclesField = cubeObstaclesGroup.getField("children")
cubeObstaclesCount = cubeObstaclesField.getCount()
cubeObstacles = []

for x in range(0, cubeObstaclesCount):
    cubeObstacles.append(cubeObstaclesField.getMFNode(x))

running = True
stopMessageSent = False
i = 0
while robot.step(timestep) != -1:
    if i < cubeObstaclesCount:
        randomlyPlaceObject(cubeObstacles[i])
        i += 1

    if running:
        time = robot.getTime()
        # If the robot has collided with something that isn't the ground or has
        # reached the goal or has even run out of time, record final time and
        # terminate simulation.
        contactPoints = thymio2.getContactPoints()
        for contact in contactPoints:
            if contact.point[2] > 0.02 or thymio2.getPosition()[1] > 3.3 or time >= 80:
                if contact.point[2] > 0.02:
                    time = 80
                running = False
                break
        robot.wwiSendText("time:%-24.3f" % time)
    else:  # Wait for record message.
        if not stopMessageSent:
            robot.wwiSendText("stop")
            stopMessageSent = True
        else:
            message = robot.wwiReceiveText()
            while message:
                if message.startswith("record:"):
                    record = robotbenchmarkRecord(message, "obstacle_avoidance", -time)
                    robot.wwiSendText(record)
                    break
                elif message == "exit":
                    break
                message = robot.wwiReceiveText()

robot.simulationSetMode(Supervisor.SIMULATION_MODE_PAUSE)
