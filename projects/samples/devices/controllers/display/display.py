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
This robot moves randomly in a maze according to its distance sensors. It has 2 display devices, one is used to display its
emotion (randomly chosen) on the screen located on its top, and the other one is used to display the camera image with extra
information (yellow blob)
"""

from controller import Robot, Camera
import random
import sys
import time


class Controller(Robot):
    def __init__(self):
        super(Controller, self).__init__()
        self.timeStep = 64
        self.camera = self.getDevice('camera')
        self.camera.enable(self.timeStep)
        self.ds0 = self.getDevice('ds0')
        self.ds0.enable(self.timeStep)
        self.ds1 = self.getDevice('ds1')
        self.ds1.enable(self.timeStep)
        self.camera_display = self.getDevice('camera_display')
        self.emoticon_display = self.getDevice('emoticon_display')
        self.left_motor = self.getDevice('left wheel motor')
        self.right_motor = self.getDevice('right wheel motor')
        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))
        self.left_motor.setVelocity(0.0)
        self.right_motor.setVelocity(0.0)

        # attach the camera with the display
        self.camera_display.attachCamera(self.camera)

        # Set the yellow color and the font once for all
        # to draw the overlay information
        self.camera_display.setColor(0xFFFF00)
        self.camera_display.setFont('Palatino Linotype', 16, True)

    def run(self):
        EMOTICON_WIDTH = 14
        EMOTICON_HEIGHT = 14
        EMOTICONS_NUMBER_X = 5
        EMOTICONS_NUMBER_Y = 11
        SPEED = 4
        MAX_SPEED = 10

        counter = 0      # time steps counter
        nop_counter = 0  # no operation counter: if positive the robot won't actuate the motors until the counter reaches 0
        random.seed(time.time())

        # import png image containing the emoticons
        emoticons_image = self.emoticon_display.imageLoad('emoticons.png')

        width = self.camera_display.getWidth()
        if self.camera.getWidth() != width:
            print('Width mismatch for camera and camera display', file=sys.stderr)
        height = self.camera_display.getHeight()
        if self.camera.getHeight() != height:
            print('Height mismatch for camera and camera display', file=sys.stderr)

        while self.step(self.timeStep) != -1:
            counter += 1
            # get the sensors values
            ds0_value = self.ds0.getValue()
            ds1_value = self.ds1.getValue()
            image = self.camera.getImage()

            # modify the emoticon every 30 steps
            if counter % 30 == 1:
                x = -EMOTICON_WIDTH * random.randrange(EMOTICONS_NUMBER_X)
                y = -EMOTICON_HEIGHT * random.randrange(EMOTICONS_NUMBER_Y)
                self.emoticon_display.imagePaste(emoticons_image, x, y, True)

            # display a rectangle around the yellow boxes
            # 1. clear the display
            self.camera_display.setAlpha(0.0)
            self.camera_display.fillRectangle(0, 0, width, height)
            self.camera_display.setAlpha(1.0)
            # 2. detect the yellow blob
            minX = width
            minY = height
            maxX = 0
            maxY = 0
            for y in range(height):
                for x in range(width):
                    r = Camera.imageGetRed(image, width, x, y)
                    g = Camera.imageGetGreen(image, width, x, y)
                    b = Camera.imageGetBlue(image, width, x, y)
                    is_yellow = r > 80 and g > 80 and b < 40 and abs(r - g) < 5
                    if is_yellow:
                        if x < minX:
                            minX = x
                        if y < minY:
                            minY = y
                        if x > maxX:
                            maxX = x
                        if y > maxY:
                            maxY = y

            # draw the blob on the display if visible
            if minX < maxX and minY < maxY:
                self.camera_display.drawRectangle(minX, minY, maxX - minX, maxY - minY)
                self.camera_display.drawText('Yellow blob', minX, minY - 20)

            # compute motor's speed with a collision avoidance
            # algorithm having a random factor for a better exploration
            if nop_counter > 0:
                nop_counter -= 1
                continue
            elif ds0_value > 100.0 and ds1_value > 100.0:  # front obstacle
                left_speed = -SPEED
                right_speed = SPEED
                nop_counter = random.randrange(20)  # give enough time to turn
            else:
                left_speed = SPEED + (ds0_value - ds1_value) / 7.0 + (random.randrange(SPEED) - SPEED / 2.0)
                left_speed = left_speed if left_speed <= MAX_SPEED else MAX_SPEED
                left_speed = left_speed if left_speed >= -MAX_SPEED else -MAX_SPEED
                right_speed = SPEED - (ds0_value - ds1_value) / 11.0 + (random.randrange(SPEED) - SPEED / 2.0)
                right_speed = right_speed if right_speed <= MAX_SPEED else MAX_SPEED
                right_speed = right_speed if right_speed >= -MAX_SPEED else -MAX_SPEED

            self.left_motor.setVelocity(left_speed)
            self.right_motor.setVelocity(right_speed)

        self.emoticon_display.imageDelete(emoticons_image)


controller = Controller()
controller.run()
