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
Simple example of motor position control.
"""

from controller import Robot


class Controller(Robot):
    def __init__(self):
        super(Controller, self).__init__()
        self.timeStep = 16
        self.linear_motor = self.getDevice('linear motor')
        self.motor = self.getDevice('motor::left finger')

    def run(self):
        while True:
            # delay
            for i in range(50):
                self.step(self.timeStep)
            # close the gripper, both sides will receive the command as the motors have the same name structure
            self.motor.setPosition(0.42)

            for i in range(50):
                self.step(self.timeStep)

            # climb the rod
            self.linear_motor.setPosition(0.14)
            self.linear_motor.setVelocity(0.1)

            for i in range(100):
                self.step(self.timeStep)

            # open the gripper
            self.motor.setPosition(0)

            for i in range(50):
                self.step(self.timeStep)

            # descend the rod
            self.linear_motor.setPosition(0)

            for i in range(100):
                self.step(self.timeStep)


controller = Controller()
controller.run()
