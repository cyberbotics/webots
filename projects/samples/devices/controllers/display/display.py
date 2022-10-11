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

"""
This robot moves randomly in a maze according to its distance sensors. It has 2 display devices, one is used to display its
emotion (randomly chosen) on the screen located on its top, and the other one is used to display the camera image with extra
information (yellow blob)
"""

from controller import Robot
from math import atan2
import sys

class Controller(Robot):
    def __init__(self):
        super(Controller, self).__init__()
        self.timeStep = 64
        self.speed = 4
        self.max_speed = 10

        self.camera = self.getDevice('camera')
        self.camera.enable(self.timeStep)

        self.ds1 = self.getDevice('ds1')
        self.ds1.enable(self.timeStep)
        self.ds2 = self.getDevice('ds2')
        self.ds2.enable(self.timeStep)

        self.camera_display = self.getDevice('camera_display')
        self.emoticon_display = self.getDevice('emoticon_display')

        self.left_motor = self.getDevice('left wheel motor')
        self.right_motor = self.getDevice('right wheel motor')
        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))
        self.left_motor.setVelocity(0.0)
        self.right_motor.setVelocity(0.0)

        # attach the camera with the display
        width = self.camera_display.getWidth()
        if self.camera.getWidth() != width:
            print('Width mismatch for camera and camera display', file=sys.stderr)
        height = self.camera_display.getHeight()
        if self.camera.getHeight() != height:
            print('Height mismatch for camera and camera display', file=sys.stderr)
        self.camera_display.attachCamera(self.camera)

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
