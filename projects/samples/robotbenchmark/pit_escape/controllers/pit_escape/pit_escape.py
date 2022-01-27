"""Sample Webots controller for the pit escape benchmark."""

from controller import Robot

robot = Robot()

timestep = int(robot.getBasicTimeStep())

# Max possible speed for the motor of the robot.
maxSpeed = 8.72

# Configuration of the main motor of the robot.
pitchMotor = robot.getDevice("body pitch motor")
pitchMotor.setPosition(float('inf'))
pitchMotor.setVelocity(0.0)

# This is the time interval between direction switches.
# The robot will start by going forward and will go backward after
# this time interval, and so on.
timeInterval = 1.5

# At first we go forward.
pitchMotor.setVelocity(maxSpeed)
forward = True
lastTime = 0

while robot.step(timestep) != -1:
    now = robot.getTime()
    # We check if enough time has elapsed.
    if now - lastTime > timeInterval:
        # If yes, then we switch directions.
        if forward:
            pitchMotor.setVelocity(-maxSpeed)
        else:
            pitchMotor.setVelocity(maxSpeed)
        forward = not forward
        lastTime = now
