"""scara_example controller."""

from controller import Robot

robot = Robot()

timeStep = int(robot.getBasicTimeStep())
t = 0
ledStatus = True

base_arm = robot.getDevice("base_arm_motor")
base_arm_pos = robot.getDevice('base_arm_position')
base_arm_pos.enable(timeStep)

arm = robot.getDevice("arm_motor")
arm_pos = robot.getDevice('arm_position')
arm_pos.enable(timeStep)

shaft = robot.getDevice("shaft_linear_motor")
led = robot.getDevice("epson_led")


while robot.step(timeStep) != -1:

    if (robot.getTime() - t > 1):
        # Read the position sensors:
        base_arm_pos_value = base_arm_pos.getValue()
        led.set(ledStatus)
        ledStatus = not(ledStatus)
        t = robot.getTime()

    arm.setPosition(0.14)
    base_arm.setPosition(0.5)
    shaft.setPosition(-0.1)
