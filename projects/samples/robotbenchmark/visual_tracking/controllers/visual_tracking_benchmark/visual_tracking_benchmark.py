"""visual tracking benchmark supervisor controller.

Manage the benchmark simulation execution and evaluate the performance
of the robot controller.
"""

from controller import Supervisor
import math
import numpy as np
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


def normalize(v):
    """Return normalized 3D vector v."""
    det = math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])
    return [v[0] / det, v[1] / det, v[2] / det]


def dotProduct(v1, v2):
    """Compute the dot product of 3D vectors v1 and v2."""
    n1 = normalize(v1)
    n2 = normalize(v2)
    return n1[0] * n2[0] + n1[1] * n2[1] + n1[2] * n2[2]


class MovingTarget():
    """Class used to manage the move of the target object."""

    SPEED = 0.0005  # target object speed in meter/milliseconds
    ROTATION_SPEED = 0.05  # target object speed in meter/milliseconds

    # File containing the target object trajectories.
    TRAJECTORY_FILE = "target_trajectory.txt"
    # Number of interpolated points used to define a trajectory.
    TRAJECTORIES_POINTS = 6

    def __init__(self, node):
        """Default constructor.

        node: target object node.
        scale: scale value of the target object.
        """
        # Get fields.
        self.node = node
        self.translationField = self.node.getField('translation')
        self.translation = self.translationField.getSFVec3f()

        self.rotationField = self.node.getField('rotation')
        self.rotationAngle = self.rotationField.getSFRotation()[3]
        self.rotationStep = float('Inf')
        self.rotationStepsCount = -1

        # Compute the trajectory based on a random combination of independent
        # trajectories written in a file.
        # Read target object trajectory from file
        # Line format: <position x>;<position z>\n
        splitTrajectories = []
        pointIndex = 0
        with open(MovingTarget.TRAJECTORY_FILE) as f:
            for line in f:
                if line.startswith('#') or line.isspace():
                    # Ignore comments.
                    continue
                if not splitTrajectories or pointIndex > MovingTarget.TRAJECTORIES_POINTS:
                    # Begin a new trajectory.
                    splitTrajectories.append([])
                    pointIndex = 0
                element = line.split(';')
                splitTrajectories[-1].append([float(element[0]), float(element[1])])
                pointIndex += 1

        self.trajectory = []
        trajectoriesCount = len(splitTrajectories)
        # Given that the np.random.seed is not set, the permutations changes
        # at each controller run.
        permutation = np.random.permutation(trajectoriesCount)
        for i in permutation:
            for point in splitTrajectories[i]:
                self.trajectory.append(point)

        self.trajectoryStep = 0

    def move(self, timestep):
        """Move the target object.

        The motion is based on the trajectory points stored in
        "target_trajectory.txt". The points position are interpolated based on
        the target motion speed to produce a smooth movement.
        Return false if the complete trajectory is executed.
        """
        if self.trajectoryStep >= len(self.trajectory):
            # return trajectory completed
            return False

        target2DPosition = self.trajectory[self.trajectoryStep]
        vector = [target2DPosition[0] - self.translation[0],
                  0.0,
                  target2DPosition[1] - self.translation[2]]
        distance = math.sqrt(vector[0] * vector[0] + vector[1] * vector[1] +
                             vector[2] * vector[2])
        maxStep = MovingTarget.SPEED * timestep

        if distance < maxStep:
            self.trajectoryStep += 1
            self.translation += vector
            segmentChanged = True
        else:
            if math.isinf(self.rotationStep):
                self.rotationStepsCount = 10
                newAngle = math.acos(dotProduct([0.0, 0.0, 1.0], vector))
                if vector[0] < 0.01:
                    newAngle = -newAngle
                diff = self.rotationAngle - newAngle
                while diff > math.pi:
                    diff -= 2 * math.pi
                while diff < -math.pi:
                    diff += 2 * math.pi
                self.rotationStep = -diff / self.rotationStepsCount

            factor = maxStep / distance
            self.translation[0] += vector[0] * factor
            self.translation[2] += vector[2] * factor
            segmentChanged = False

        self.translationField.setSFVec3f(self.translation)

        if self.rotationStepsCount > 0:
            if segmentChanged:
                self.rotationAngle += self.rotationStep * \
                    self.rotationStepsCount
                self.rotationStepsCount = 0
            else:
                self.rotationAngle += self.rotationStep
                self.rotationStepsCount -= 1
            self.rotationField.setSFRotation([0.0, 1.0, 0.0,
                                              self.rotationAngle])

        if segmentChanged:
            self.rotationStep = float('Inf')
        return True

    def hit(self, origin, sightVector, hitError):
        """Return if the target object lies on the sight line.

        The sight line is defined by the origin point and sightVector.
        hitError specifies the difference errors in meters that can be
        considered as an hit.
        """
        distance = [0, 0, 0]
        for i in range(0, 3):
            distance[i] = self.translation[i] - origin[i]
        v1 = normalize(distance)
        v2 = normalize(sightVector)
        return abs(v1[0] - v2[0]) < hitError and \
            abs(v1[1] - v2[1]) < hitError and abs(v1[2] - v2[2]) < hitError


# Parse controller arguments.
hitError = 0.1
for arg in sys.argv:
    if arg.startswith('hit-error='):
        hitError = float(arg[len('hit-error='):])

# Create the Supervisor instance.
robot = Supervisor()

# Get the time step of the current world.
timestep = int(robot.getBasicTimeStep() * 4)
robot.wwiSendText("setup:%.2f;%.2f" % (hitError, timestep))

# Create instance of moving target object.
target = MovingTarget(robot.getFromDef('TARGET'))

robotHead = robot.getFromDef('HEAD_CAM')

hitsCount = 0
stepsCount = 0
isRunning = True
# Main loop:
# perform simulation steps until Webots is stopping the controller or
# the tartet object trajectory is completed.
while robot.step(timestep) != -1 and isRunning:
    # Evaluate the precision of the tracked object position by checking if the
    # camera is looking at the target object direction.
    # Camera local orientation is [0, 0 -1], thus the global orietation is
    # R * [0, 0, -1], where R is the robot head global rotation matrix.
    R = robotHead.getOrientation()
    hitsCount += target.hit(robotHead.getPosition(), [-R[2], -R[5], -R[8]],
                            hitError)
    stepsCount += 1
    robot.wwiSendText("hits:%d/%d" % (hitsCount, stepsCount))

    # Update target object position:
    # return if the whole trajectory has been completed.
    isRunning = target.move(timestep)

# Compute final grade and exit
if stepsCount == 0:
    hitRate = 0.0
else:
    hitRate = float(hitsCount) / stepsCount

robot.wwiSendText("stop")

# Wait for record message.
timestep = int(robot.getBasicTimeStep())
while robot.step(timestep) != -1:
    message = robot.wwiReceiveText()
    if message:
        if message.startswith("record:"):
            record = robotbenchmarkRecord(message, "visual_tracking", hitRate)
            robot.wwiSendText(record)
            break
        elif message == "exit":
            break

robot.simulationSetMode(Supervisor.SIMULATION_MODE_PAUSE)
