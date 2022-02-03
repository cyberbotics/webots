"""scara_example controller."""

from controller import Robot

robot = Robot()

timestep = int(robot.getBasicTimeStep())
    
base_arm = robot.getDevice("base_arm_motor")
base_arm_pos = robot.getDevice('base_arm_position_sensor')
base_arm_pos.enable(timestep)

arm = robot.getDevice("arm_motor")
arm_pos = robot.getDevice('arm_position_sensor')
arm_pos.enable(timestep)

shaft = robot.getDevice("shaft_linear_motor")

while robot.step(timestep) != -1:

    # Read the position sensors:
    base_arm_pos_value = base_arm_pos.getValue()
    
    arm.setPosition(0.14)
    base_arm.setPosition(1.4)
    shaft.setPosition(-0.1)