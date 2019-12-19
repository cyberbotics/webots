"""Supervisor of the Maze Runner benchmark."""

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


def isPositionChanged(v1, v2):
    return abs(v1[0] - v2[0]) > 0.001 or abs(v1[2] - v2[2]) > 0.001


def isMazeEndReached(position):
    return position[0] < 0.60 and position[0] > 0.45 and position[2] < 0.15 and position[2] > -0.15


robot = Supervisor()
timestep = int(4 * robot.getBasicTimeStep())
robot.step(10 * timestep)

thymio2 = robot.getFromDef("THYMIO2")

robot.step(10 * timestep)

mazeBlocksList = []
mazeBlocksListCount = 0
topChildrenField = robot.getFromDef("MAZE_WALLS").getField("children")
topNodesCount = topChildrenField.getCount()
for i in range(topNodesCount):
    node = topChildrenField.getMFNode(i)
    if node.getTypeName() == "MazeBlock":
        object = {
            "node": node,
            "initialPosition": node.getPosition()
        }
        mazeBlocksList.append(object)
        mazeBlocksListCount += 1


running = True
stopMessageSent = False
while robot.step(timestep) != -1:
    if running:
        time = robot.getTime()
        # If some of the maze blocks have been moved immediately terminate the benchmark
        for i in range(0, mazeBlocksListCount):
            item = mazeBlocksList[i]
            if isPositionChanged(item['initialPosition'], item['node'].getPosition()):
                time = 60
                robot.wwiSendText("time:%-24.3f" % time)
                break

        if time < 60 and not isMazeEndReached(thymio2.getPosition()):
            robot.wwiSendText("time:%-24.3f" % time)
        else:
            running = False
            timestep = int(timestep / 4)

    else:  # Wait for record message.
        if not stopMessageSent:
            robot.wwiSendText("stop")
            stopMessageSent = True
        else:
            message = robot.wwiReceiveText()
            if message:
                if message.startswith("record:"):
                    record = robotbenchmarkRecord(message, "maze_runner", -time)
                    robot.wwiSendText(record)
                    break
                elif message == "exit":
                    break

robot.simulationSetMode(Supervisor.SIMULATION_MODE_PAUSE)
