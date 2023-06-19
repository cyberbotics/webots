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
Demo/Test for the Compass device
A robot is equipped with a needle that constantly point towards the north while the moves around.
"""

from controller import Robot
from math import atan2


class Controller(Robot):
    def __init__(self):
        super(Controller, self).__init__()
        self.timeStep = 8
        self.compass = self.getDevice('compass')
        self.compass.enable(self.timeStep)
        self.arrow = self.getDevice('arrow')
        self.us0 = self.getDevice('us0')
        self.us0.enable(self.timeStep)
        self.us1 = self.getDevice('us1')
        self.us1.enable(self.timeStep)
        self.left_motor = self.getDevice('left wheel motor')
        self.right_motor = self.getDevice('right wheel motor')
        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))

    def run(self):
        while self.step(self.timeStep) != -1:
            # read distance sensors
            d0 = self.us0.getValue()
            d1 = self.us1.getValue()
            if d0 < 100 or d1 < 100:  # in case of collision turn left
                self.left_motor.setVelocity(-5)
                self.right_motor.setVelocity(5)
            else:  # otherwise go straight
                self.left_motor.setVelocity(5)
                self.right_motor.setVelocity(5)
            # read compass and rotate arrow accordingly
            north = self.compass.getValues()
            angle = atan2(north[1], north[0])
            self.arrow.setPosition(angle)


controller = Controller()
controller.run()
