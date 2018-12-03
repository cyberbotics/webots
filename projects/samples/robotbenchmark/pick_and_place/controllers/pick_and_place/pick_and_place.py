"""Sample Webots controller for the pick and place benchmark."""

from controller import Robot

# Create the Robot instance.
robot = Robot()

# Get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# Inizialize base motors.
wheels = []
wheels.append(robot.getMotor("wheel1"))
wheels.append(robot.getMotor("wheel2"))
wheels.append(robot.getMotor("wheel3"))
wheels.append(robot.getMotor("wheel4"))
for wheel in wheels:
    # Activate controlling the motors setting the velocity.
    # Otherwise by default the motor expects to be controlled in force or position,
    # and setVelocity will set the maximum motor velocity instead of the target velocity.
    wheel.setPosition(float('+inf'))

# Initialize arm motors.
armMotors = []
armMotors.append(robot.getMotor("arm1"))
armMotors.append(robot.getMotor("arm2"))
armMotors.append(robot.getMotor("arm3"))
armMotors.append(robot.getMotor("arm4"))
armMotors.append(robot.getMotor("arm5"))
# Set the maximum motor velocity.
armMotors[0].setVelocity(0.2)
armMotors[1].setVelocity(0.5)
armMotors[2].setVelocity(0.5)
armMotors[3].setVelocity(0.3)

# Initialize arm position sensors.
# These sensors can be used to get the current joint position and monitor the joint movements.
armPositionSensors = []
armPositionSensors.append(robot.getPositionSensor("arm1sensor"))
armPositionSensors.append(robot.getPositionSensor("arm2sensor"))
armPositionSensors.append(robot.getPositionSensor("arm3sensor"))
armPositionSensors.append(robot.getPositionSensor("arm4sensor"))
armPositionSensors.append(robot.getPositionSensor("arm5sensor"))
for sensor in armPositionSensors:
    sensor.enable(timestep)

# Initialize gripper motors.
finger1 = robot.getMotor("finger1")
finger2 = robot.getMotor("finger2")
# Set the maximum motor velocity.
finger1.setVelocity(0.03)
finger2.setVelocity(0.03)
# Read the miminum and maximum position of the gripper motors.
fingerMinPosition = finger1.getMinPosition()
fingerMaxPosition = finger1.getMaxPosition()

# Move forward.
for wheel in wheels:
    wheel.setVelocity(7.0)
# Wait until the robot is in front of the box.
robot.step(520 * timestep)

# Stop moving forward.
for wheel in wheels:
    wheel.setVelocity(0.0)

# Move arm and open gripper.
armMotors[1].setPosition(-0.55)
armMotors[2].setPosition(-0.9)
armMotors[3].setPosition(-1.5)
finger1.setPosition(fingerMaxPosition)
finger2.setPosition(fingerMaxPosition)

# Monitor the arm joint position to detect when the motion is completed.
while robot.step(timestep) != -1:
    if abs(armPositionSensors[3].getValue() - (-1.2)) < 0.01:
        # Motion completed.
        break

# Close gripper.
finger1.setPosition(0.013)
finger2.setPosition(0.013)
# Wait until the gripper is closed.
robot.step(50 * timestep)

# Lift arm.
armMotors[1].setPosition(0)
# Wait until the arm is lifted.
robot.step(200 * timestep)

# Rotate the robot.
wheels[0].setVelocity(2.5)
wheels[1].setVelocity(-2.5)
wheels[2].setVelocity(2.5)
wheels[3].setVelocity(-2.5)
# Wait for a fixed amount to step that the robot rotates.
robot.step(690 * timestep)

# Move forward.
wheels[1].setVelocity(2.5)
wheels[3].setVelocity(2.5)
robot.step(900 * timestep)

# Rotate the robot.
wheels[0].setVelocity(1.0)
wheels[1].setVelocity(-1.0)
wheels[2].setVelocity(1.0)
wheels[3].setVelocity(-1.0)
robot.step(200 * timestep)

# Move forward.
wheels[1].setVelocity(1.0)
wheels[3].setVelocity(1.0)
robot.step(300 * timestep)

# Rotate the robot.
wheels[0].setVelocity(1.0)
wheels[1].setVelocity(-1.0)
wheels[2].setVelocity(1.0)
wheels[3].setVelocity(-1.0)
robot.step(130 * timestep)

# Move forward.
wheels[1].setVelocity(1.0)
wheels[3].setVelocity(1.0)
robot.step(310 * timestep)

# Stop.
for wheel in wheels:
    wheel.setVelocity(0.0)

# Move arm down
armMotors[3].setPosition(0)
armMotors[2].setPosition(-0.3)
robot.step(200 * timestep)

armMotors[1].setPosition(-1.0)
robot.step(200 * timestep)

armMotors[3].setPosition(-1.0)
robot.step(200 * timestep)

armMotors[2].setPosition(-0.4)
robot.step(50 * timestep)

# Open gripper.
finger1.setPosition(fingerMaxPosition)
finger2.setPosition(fingerMaxPosition)
robot.step(50 * timestep)
