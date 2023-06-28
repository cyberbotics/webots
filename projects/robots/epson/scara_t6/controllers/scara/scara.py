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

""" scara example controller."""

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
        ledStatus = not ledStatus
        t = robot.getTime()

    arm.setPosition(0.14)
    base_arm.setPosition(0.5)
    shaft.setPosition(-0.1)
