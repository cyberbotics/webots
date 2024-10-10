# Copyright 1996-2023 Cyberbotics Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from controller import Robot, Keyboard

# Create the Robot instance.
robot = Robot()

keyboard = Keyboard()
keyboard.enable(1000)

# Get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# Get left and right motors.
left_motor = robot.getDevice('left_motor')
right_motor = robot.getDevice('right_motor')

# Enable position control mode for motors.
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))

# Set initial velocities.
left_velocity = 0.0
right_velocity = 0.0

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    # Read keyboard input.
    key = keyboard.getKey()
    # Process keyboard input.
    if key == ord('W') or key == ord('w'):
        left_velocity = 5.0
        right_velocity = 5.0
    elif key == ord('S') or key == ord('s'):
        left_velocity = -5.0
        right_velocity = -5.0
    elif key == ord('A') or key == ord('a'):
        left_velocity = -2.5
        right_velocity = 2.5
    elif key == ord('D') or key == ord('d'):
        left_velocity = 2.5
        right_velocity = -2.5
    else:
        left_velocity = 0.0
        right_velocity = 0.0
    # Set motor velocities.
    left_motor.setVelocity(left_velocity)
    right_motor.setVelocity(right_velocity)
