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

print('Move forward until an obstabcle is detected.')

robot.getDevice('motor led').set(0xFF0000)
robot.getDevice('distance sensor led').set(0x00FF00)

motor = robot.getDevice('motor')
motor.setPosition(float('inf'))  # Velocity control mode.
motor.setVelocity(0.5 * motor.getMaxVelocity())

distanceSensor = robot.getDevice('distance sensor')
distanceSensor.enable(timestep)


while robot.step(timestep) != -1:
    distance = distanceSensor.getValue()
    if distance < 100:
        print('Obstacle detected. Stop the motor.')
        motor.setVelocity(0)
        robot.step(timestep)
        break
