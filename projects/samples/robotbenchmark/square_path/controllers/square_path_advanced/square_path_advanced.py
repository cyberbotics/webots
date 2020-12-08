"""square_path_advanced controller."""

import sys

from controller import Robot
from math import pi

# ratio between the value reported by the wheel sensor and the actual
# distance traveled by the robot while going in a straight line, in meters.
TRANSLATE_FACTOR = 0.096

# ratio between the value reported by the wheel sensor and the actual
# angle traveled by the robot while turning, in radian.
ROTATE_FACTOR = 0.566

# maximum speed for the velocity value of the wheels.
MAX_SPEED = 5.24

# minimum speed used while slowing down and trying to stop at a precise
# location.
MIN_SPEED = 0.1

robot = Robot()

timestep = int(robot.getBasicTimeStep())

leftWheel = robot.getDevice('left wheel')
rightWheel = robot.getDevice('right wheel')

leftWheel.setVelocity(0)
rightWheel.setVelocity(0)

leftWheel.setPosition(float('inf'))
rightWheel.setPosition(float('inf'))

leftWheelSensor = robot.getDevice('left wheel sensor')
leftWheelSensor.enable(timestep)

rightWheelSensor = robot.getDevice('right wheel sensor')
rightWheelSensor.enable(timestep)


def setMotors(speed, rotation):
    """
    Set the velocity value of both wheels.

    If rotation is false, both wheels will have the same velocity set,
    which will make the robot go straight. Otherwise, the left wheel will be
    set with the opposite value, making the robot turn without changing
    position.
    """
    leftWheel.setVelocity(-speed if rotation else speed)
    rightWheel.setVelocity(speed)


def move(value, threshold, rotation):
    """
    Move the robot on a straight line or rotates it around itself.

    value: distance in meters or angle in radian depending on the value of
    rotation.
    threshold: the robot will slow down when reaching this value.
    rotation: False to move the robot on a straight line for a certain.
    distance, or True to rotate the robot for a certain angle.
    """
    factor = (ROTATE_FACTOR if rotation else TRANSLATE_FACTOR)

    # recovers the current value of the sensors.
    rightStart = rightWheelSensor.getValue() * factor
    leftStart = leftWheelSensor.getValue() * factor

    # computes the value we want to reach for each wheel.
    rightGoal = rightStart + value
    leftGoal = leftStart + (-value if rotation else value)

    # computes in which direction the robot needs to go.
    if value > 0:
        sign = 1
        absValue = value
    elif value < 0:
        sign = -1
        absValue = -value
    else:
        return

    # loops until the robot has reached the destination.
    while (robot.step(timestep) != -1 and absValue > 0):
        # full speed if threshold hasn't been reached.
        if absValue > threshold:
            speed = MAX_SPEED
        # otherwise we follow a linear decay down to a minimum
        # (so the robot doesn't completely stop before reaching the goal).
        else:
            speed = MAX_SPEED * (absValue / threshold)

        if speed < MIN_SPEED:
            speed = MIN_SPEED

        # applies the velocity to the wheels.
        setMotors(sign * speed, rotation)

        # updates distance remaining using the wheel sensors.
        rightValue = sign * (rightGoal - rightWheelSensor.getValue() * factor)
        leftValue = sign * (leftGoal - leftWheelSensor.getValue() * factor)

        if rotation:
            leftValue = -leftValue

        absValue = (leftValue + rightValue) / 2

    # stops the robot when we are done.
    setMotors(0, False)


def forward(distance):
    """Make the robot move forward or backward for a certain distance."""
    move(distance, 0.5, False)


def rotate(angle):
    """Make the robot rotate for a certain angle."""
    move(angle, pi / 4, True)


# first argument is the side of one edge of the square, in meters.
if len(sys.argv) > 1:
    side = float(sys.argv[1])
else:
    side = 2

# performs the first step of simulation, so the sensors have a valid value.
robot.step(timestep)

# moves the robot along the square.
for x in range(0, 3):
    forward(side)
    rotate(-pi / 2)

# we don't need to turn for the last edge.
forward(side)
