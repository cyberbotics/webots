# Copyright 1996-2021 Cyberbotics Ltd.
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

from webots import Robot, DistanceSensor, Motor

robot = Robot()
ds0 = DistanceSensor('ds0')
ds1 = DistanceSensor('ds1')
ds0.enable(64)
ds1.enable(64)
left_motor = Motor('left wheel motor')
right_motor = Motor('right wheel motor')
left_motor.position = float('inf')
right_motor.position = float('inf')
left_motor.velocity = 0.0
right_motor.velocity = 0.0

while (robot.step(64) != -1):
    if ds1.value > 500:
        if ds0.value > 500:
            left_speed = -6
            right_speed = -3
        else:
            left_speed = -ds1.value / 100
            right_speed = (ds0.value / 100) + 0.5
    elif ds0.value > 500:
        left_speed = (ds1.value / 100) + 0.5
        right_speed = -ds0.value / 100
    else:
        left_speed = 6
        right_speed = 6
    left_motor.velocity = left_speed
    right_motor.velocity = right_speed
