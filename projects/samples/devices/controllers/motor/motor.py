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

from controller import Robot, AnsiCodes
import math


class Controller(Robot):
    def __init__(self):
        super(Controller, self).__init__()
        self.timeStep = int(self.getBasicTimeStep())
        self.motor = self.getDevice('motor')
        self.motor.enableTorqueFeedback(self.timeStep)
        self.batterySensorEnable(self.timeStep)

    def run(self):
        target = 0
        counter = 0
        while self.step(self.timeStep) != -1:
            self.motor.setPosition(target)
            if counter == 50:
                target += math.pi / 4
                counter = 0
            counter += 1
            print(AnsiCodes.CLEAR_SCREEN)
            print(f'Force feedback = {self.motor.getTorqueFeedback()}')
            print(f'Battery level  = {self.batterySensorGetValue()}')


controller = Controller()
controller.run()
