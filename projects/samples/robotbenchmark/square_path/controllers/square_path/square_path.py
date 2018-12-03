"""Sample Webots controller for the square path benchmark."""

from controller import Robot

# Get pointer to the robot.
robot = Robot()

# Get pointer to each wheel of our robot.
leftWheel = robot.getMotor('left wheel')
rightWheel = robot.getMotor('right wheel')

# Repeat the following 4 times (once for each side).
for i in range(0, 4):
    # First set both wheels to go forward, so the robot goes straight.
    leftWheel.setPosition(1000)
    rightWheel.setPosition(1000)
    # Wait for the robot to reach a corner.
    robot.step(3900)
    # Then, set the right wheel backward, so the robot will turn right.
    leftWheel.setPosition(1000)
    rightWheel.setPosition(-1000)
    # Wait until the robot has turned 90 degrees clockwise.
    robot.step(480)

# Stop the robot when path is completed, as the robot performance
# is only computed when the robot has stopped.
leftWheel.setVelocity(0)
rightWheel.setVelocity(0)
