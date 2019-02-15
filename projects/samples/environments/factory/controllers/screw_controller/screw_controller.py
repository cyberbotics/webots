"""screw_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, LED, DistanceSensor
from controller import Robot

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

motor = robot.getMotor('linear motor')
 
positionSensor = robot.getPositionSensor('position sensor')
positionSensor.enable(timestep)

while robot.step(timestep) != -1:
    targetPosition = positionSensor.getValue() * 0.001
    maxPosition = motor.getMaxPosition()
    minPosition = motor.getMinPosition()
    targetPosition = max(min(targetPosition, maxPosition), minPosition)
    motor.setPosition(targetPosition)
    