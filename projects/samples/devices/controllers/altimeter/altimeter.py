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
Use altimeter to control robot's ascent and descent of ramp.
"""

from controller import Robot


class Controller(Robot):
    def __init__(self):
        super(Controller, self).__init__()
        self.timeStep = int(self.getBasicTimeStep())
        self.altimeter = self.getDevice('altimeter')
        self.altimeter.enable(self.timeStep)
        self.left_motor = self.getDevice('left wheel motor')
        self.right_motor = self.getDevice('right wheel motor')
        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))

    def run(self):
        direction_switch = False
        while self.step(self.timeStep) != -1:
            altitude = self.altimeter.getValue()
            if not direction_switch:
                self.left_motor.setVelocity(2.0)
                self.right_motor.setVelocity(2.0)
                if altitude <= 0.05:
                    direction_switch = True
            else:
                self.left_motor.setVelocity(-2.0)
                self.right_motor.setVelocity(-2.0)
                if altitude >= 0.25:
                    direction_switch = False


controller = Controller()
controller.run()
