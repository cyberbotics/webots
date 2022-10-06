# Copyright 1996-2022 Cyberbotics Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from controller import Robot, Motor

robot = Robot()
ds0 = robot.getDevice('ds0')
ds1 = robot.getDevice('ds1')
timeStep = int(robot.getBasicTimeStep())
ds0.enable(timeStep)
ds1.enable(timeStep)
left_motor = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))
left_motor.setVelocity(0)
right_motor.setVelocity(0)

print('left motor type is ' + 'rotational' if left_motor.getType() == Motor.ROTATIONAL else 'linear' + '.')
while (robot.step(timeStep) != -1):
    if ds1.getValue() > 500:
        if ds0.getValue() > 500:
            left_speed = -6
            right_speed = -3
        else:
            left_speed = -ds1.getValue() / 100
            right_speed = (ds0.getValue() / 100) + 0.5
    elif ds0.getValue() > 500:
        left_speed = (ds1.getValue() / 100) + 0.5
        right_speed = -ds0.getValue() / 100
    else:
        left_speed = 6
        right_speed = 6
    left_motor.setVelocity(left_speed)
    right_motor.setVelocity(right_speed)
