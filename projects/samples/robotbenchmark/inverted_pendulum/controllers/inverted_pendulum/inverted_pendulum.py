"""Sample Webots controller for the inverted pendulum benchmark."""

from controller import Robot
import math

# Get pointer to the robot.
robot = Robot()

# Get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# Get pointers to the position sensor and enable it.
ps = robot.getDevice('pendulum sensor')
ps.enable(timestep)

# Get pointers to the motors and set target position to infinity (speed control).
leftMotor = robot.getDevice("left wheel motor")
rightMotor = robot.getDevice("right wheel motor")
leftMotor.setPosition(float('+inf'))
rightMotor.setPosition(float('+inf'))
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)
maxSpeed = min(rightMotor.getMaxVelocity(), leftMotor.getMaxVelocity())

# Define the PID control constants and variables.
KP = 31.4
KI = 100.5
KD = 0
integral = 0.0
previous_position = 0.0

# Initialize the robot speed (left wheel, right wheel).
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)

# Main loop: perform a simulation step until the simulation is over.
while robot.step(timestep) != -1:
    # Read the sensor measurement.
    position = ps.getValue()

    # Stop the robot when the pendulum falls.
    if math.fabs(position) > math.pi * 0.5:
        leftMotor.setVelocity(0.0)
        rightMotor.setVelocity(0.0)
        break

    # PID control.
    integral = integral + (position + previous_position) * 0.5 / timestep
    derivative = (position - previous_position) / timestep
    speed = KP * position + KI * integral + KD * derivative

    # Clamp speed to the maximum speed.
    if speed > maxSpeed:
        speed = maxSpeed
    elif speed < -maxSpeed:
        speed = -maxSpeed

    # Set the robot speed (left wheel, right wheel).
    leftMotor.setVelocity(-speed)
    rightMotor.setVelocity(-speed)

    # Store previous position for the next controller step.
    previous_position = position
