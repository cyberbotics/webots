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

"""
An example of a controller using a light sensor device.
"""

from controller import Robot


class Controller(Robot):
    timeStep = 64

    def __init__(self):
        super(Controller, self).__init__()
        # get a handler to the distance sensors.
        self.ls0 = self.getDevice('ls0')
        self.ls1 = self.getDevice('ls1')
        self.ls0.enable(self.timeStep)
        self.ls1.enable(self.timeStep)
        self.left_motor = self.getDevice('left wheel motor')
        self.right_motor = self.getDevice('right wheel motor')
        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))
        self.left_motor.setVelocity(0.5)
        self.right_motor.setVelocity(-0.5)

    def run(self):
        MAX_SPEED = 10
        print('Move the light (shift + drag mouse), the robot should follow it.')
        while self.step(self.timeStep) != -1:
            # read sensor values
            ls0_value = self.ls0.getValue()
            ls1_value = self.ls1.getValue()
            left_speed = (1024 - ls0_value) / 100.0
            left_speed = left_speed if left_speed < MAX_SPEED else MAX_SPEED
            right_speed = (1024 - ls1_value) / 100.0
            right_speed = right_speed if right_speed < MAX_SPEED else MAX_SPEED

            # Set the motor speeds
            self.left_motor.setVelocity(left_speed)
            self.right_motor.setVelocity(right_speed)


controller = Controller()
controller.run()
