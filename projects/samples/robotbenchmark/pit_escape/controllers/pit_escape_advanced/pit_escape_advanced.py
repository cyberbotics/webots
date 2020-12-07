"""Advanced Webots controller for the pit escape benchmark."""

from controller import Robot

robot = Robot()

timestep = int(robot.getBasicTimeStep())

pitchMotor = robot.getDevice("body pitch motor")
pitchMotor.setPosition(float('inf'))
pitchMotor.setVelocity(0.0)

bodyGyro = robot.getDevice("body gyro")
bodyGyro.enable(timestep)

maxSpeed = 8.72

pitchMotor.setVelocity(maxSpeed)

lastValues = bodyGyro.getValues()

while robot.step(timestep) != -1:
    bodyValues = bodyGyro.getValues()

    if lastValues[0] >= 0 and bodyValues[0] < 0:
        pitchMotor.setVelocity(-maxSpeed)
    elif lastValues[0] < 0 and bodyValues[0] >= 0:
        pitchMotor.setVelocity(maxSpeed)

    lastValues = bodyValues
