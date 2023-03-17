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

from controller import Robot

robot = Robot()
timestep = int(robot.getBasicTimeStep())

print('Move forward and avoid obstabcles.')

leftMotor = robot.getDevice('left motor')
robot.getDevice('left motor led').set(0xFFFF00)
leftMotor.setPosition(float('inf'))  # Velocity control mode.
rightMotor = robot.getDevice('right motor')
robot.getDevice('right motor led').set(0x00FFFF)
rightMotor.setPosition(float('inf'))  # Velocity control mode.
maxSpeed = leftMotor.getMaxVelocity()
cruisingSpeed = 0.5 * maxSpeed

leftDistanceSensor = robot.getDevice('left distance sensor')
leftDistanceSensor.enable(timestep)
robot.getDevice('left distance sensor led').set(0xFF0000)
rightDistanceSensor = robot.getDevice('right distance sensor')
rightDistanceSensor.enable(timestep)
robot.getDevice('right distance sensor led').set(0x00FF00)

while robot.step(timestep) != -1:
    # Invert the TinkerbotsDistanceSensor.lookupTable to have an appoximation of the distance in meters.
    rightDistance = 128 - rightDistanceSensor.getValue()
    leftDistance = 128 - leftDistanceSensor.getValue()
    delta = leftDistance - rightDistance
    leftSpeed = -cruisingSpeed - delta + 0.5
    rightSpeed = cruisingSpeed - delta + 0.5
    leftSpeed = max(-maxSpeed, min(maxSpeed, leftSpeed))
    rightSpeed = max(-maxSpeed, min(maxSpeed, rightSpeed))
    leftMotor.setVelocity(leftSpeed)
    rightMotor.setVelocity(rightSpeed)
