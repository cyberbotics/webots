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
An example of use of a camera device.
"""

from controller import Robot, AnsiCodes


class Controller(Robot):
    def __init__(self):
        super(Controller, self).__init__()
        self.timeStep = int(self.getBasicTimeStep())
        self.camera = self.getDevice('camera')
        self.camera.enable(self.timeStep)
        self.width = self.camera.getWidth()
        self.height = self.camera.getHeight()
        self.left_motor = self.getDevice('left wheel motor')
        self.right_motor = self.getDevice('right wheel motor')
        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))
        self.left_motor.setVelocity(0.0)
        self.right_motor.setVelocity(0.0)

    def run(self):
        speed = 4
        pause_counter = 0
        left_speed = 0
        right_speed = 0
        red = 0
        green = 0
        blue = 0
        while self.step(self.timeStep) != -1:
            image = self.camera.getImage()
            if pause_counter > 0:
                pause_counter -= 1
            # Case 1
            # A blob was found recently
            # The robot waits in front of it until pause_counter
            # is decremented enough
            if pause_counter > 640 / self.timeStep:
                left_speed = 0
                right_speed = 0
            # Case 2
            # A blob was found quite recently
            # The robot begins to turn but don't analyse the image for a while,
            # otherwise the same blob would be found again
            elif pause_counter > 0:
                left_speed = -speed
                right_speed = speed
            # Case 3
            # The robot turns and analyse the camera image in order
            # to find a new blob
            elif not image:  # image may be NULL if Robot.synchronization is FALSE
                left_speed = 0
                right_speed = 0
            else:  # pause_counter == 0
                red = 0;
                green = 0;
                blue = 0;
                # Here we analyse the image from the camera. The goal is to detect a
                # blob (a spot of color) of a defined color in the middle of our
                # screen.
                # In order to achieve that we simply parse the image pixels of the
                # center of the image, and sum the color components individually
                for (i = width / 3; i < 2 * width / 3; i+=1):
                    for (j = height / 2; j < 3 * height / 4; j++):
                        red += wb_camera_image_get_red(image, width, i, j)
                        blue += wb_camera_image_get_blue(image, width, i, j)
                        green += wb_camera_image_get_green(image, width, i, j)



            print(AnsiCodes.RED_FOREGROUND + str(image[0]) + AnsiCodes.RESET)


controller = Controller()
controller.run()
