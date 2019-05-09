"""Naive maze runner controller."""

from controller import Robot

# Get reference to the robot.
robot = Robot()

# Get simulation step length.
timeStep = int(robot.getBasicTimeStep())

# Constants of the Thymio II motors and distance sensors.
maxMotorVelocity = 6

# Get left and right wheel motors.
leftMotor = robot.getMotor("motor.left")
rightMotor = robot.getMotor("motor.right")

# Frontal distance sensors that can be use to detect the walls.
outerLeftSensor = robot.getDistanceSensor("prox.horizontal.0")
centralLeftSensor = robot.getDistanceSensor("prox.horizontal.1")
centralSensor = robot.getDistanceSensor("prox.horizontal.2")
centralRightSensor = robot.getDistanceSensor("prox.horizontal.3")
outerRightSensor = robot.getDistanceSensor("prox.horizontal.4")

# Enable sensors.
outerLeftSensor.enable(timeStep)
centralLeftSensor.enable(timeStep)
centralSensor.enable(timeStep)
centralRightSensor.enable(timeStep)
outerRightSensor.enable(timeStep)

# Get and enable ground sensors to detect the black circles.
# groundLeftSensor = robot.getDistanceSensor("prox.ground.0")
# groundRightSensor = robot.getDistanceSensor("prox.ground.1")
# groundLeftSensor.enable(timeStep)
# groundRightSensor.enable(timeStep)

# Disable motor PID control mode.
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))

# Set ideal motor velocity.
velocity = 0.7 * maxMotorVelocity

isRotating = False
while robot.step(timeStep) != -1:
    # Read values from four distance sensors.
    if not isRotating and centralSensor.getValue() > 3500:
        # Black circle detected.
        isRotating = True
    elif isRotating and outerLeftSensor.getValue() == 0:
        isRotating = False

    leftMotor.setVelocity(velocity)
    if isRotating:
        rightMotor.setVelocity(-velocity)
    else:
        rightMotor.setVelocity(velocity)
