"""led_controller controller."""

from controller import Robot

robot = Robot()

timestep = int(robot.getBasicTimeStep())

led = robot.getLED('led')
led.set(True)

positionSensor = robot.getPositionSensor('emergency button sensor')
positionSensor.enable(timestep)

released = True

while robot.step(timestep) != -1:
    value = positionSensor.getValue()
    if value > -0.002:
        released = True

    if released and value < -0.010:
        released = False
        led.set(not led.get())
