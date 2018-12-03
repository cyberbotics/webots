"""Sample Webots controller for the wall following benchmark."""

from controller import Robot


def getDistance(sensor):
    """
    Return the distance of an obstacle for a sensor.

    The value returned by the getValue() method of the distance sensors
    corresponds to a physical value (here we have a sonar, so it is the
    strength of the sonar ray). This function makes a conversion to a
    distance value in meters.
    """
    return ((1000 - sensor.getValue()) / 1000) * 5


# Maximum speed for the velocity value of the wheels.
# Don't change this value.
MAX_SPEED = 5.24

# Get pointer to the robot.
robot = Robot()

# Get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# Get pointer to the robot wheels motors.
leftWheel = robot.getMotor('left wheel')
rightWheel = robot.getMotor('right wheel')

# We will use the velocity parameter of the wheels, so we need to
# set the target position to infinity.
leftWheel.setPosition(float('inf'))
rightWheel.setPosition(float('inf'))

# Get and enable the distance sensors.
frontSensor = robot.getDistanceSensor("so3")
frontSensor.enable(timestep)
sideSensor = robot.getDistanceSensor("so0")
sideSensor.enable(timestep)

# Move forward until we are 50 cm away from the wall.
leftWheel.setVelocity(MAX_SPEED)
rightWheel.setVelocity(MAX_SPEED)
while robot.step(timestep) != -1:
    if getDistance(frontSensor) < 0.5:
        break

# Rotate clockwise until the wall is to our left.
leftWheel.setVelocity(MAX_SPEED)
rightWheel.setVelocity(-MAX_SPEED)
while robot.step(timestep) != -1:
    # Rotate until there is a wall to our left, and nothing in front of us.
    if getDistance(sideSensor) < 1:
        break

# Main loop.
while robot.step(timestep) != -1:

    # Too close to the wall, we need to turn right.
    if getDistance(sideSensor) < 0.4:
        leftWheel.setVelocity(MAX_SPEED)
        rightWheel.setVelocity(MAX_SPEED * 0.9)

    # Too far from the wall, we need to turn left.
    elif getDistance(sideSensor) > 0.6:
        leftWheel.setVelocity(MAX_SPEED * 0.9)
        rightWheel.setVelocity(MAX_SPEED)

    # We are in the right direction.
    else:
        leftWheel.setVelocity(MAX_SPEED)
        rightWheel.setVelocity(MAX_SPEED)

# Stop the robot when we are done.
leftWheel.setVelocity(0)
rightWheel.setVelocity(0)
