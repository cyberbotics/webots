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
An example of use of a range-finder device.
"""

from controller import Robot, RangeFinder
import os
if os.name == 'nt':
    from ctypes import create_unicode_buffer, windll


class Controller(Robot):
    def __init__(self):
        super(Controller, self).__init__()
        self.timeStep = 64
        self.range_finder = self.getDevice('range-finder')
        self.range_finder.enable(self.timeStep)
        self.left_motor = self.getDevice('left wheel motor')
        self.right_motor = self.getDevice('right wheel motor')
        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))
        self.left_motor.setVelocity(0)
        self.right_motor.setVelocity(0)

    def run(self):
        SPEED = 4
        state = 'SEARCHING'
        save_image = True
        width = self.range_finder.getWidth()
        height = self.range_finder.getHeight()
        while self.step(self.timeStep) != -1:
            image = self.range_finder.getRangeImage()
            average_distance = 0
            minimal_distance = 100
            # Here we analyse the image we've got from the range-finder. Our robot has two
            # states: SEARCHING and MOVING. When it is in SEARCHING, it looks around
            # for a direction in which it has enough distance to move. So in this
            # case we compute the weighted average distance of the range-finder. In the
            # MOVING state, it is moving straight until it comes too close from an
            # obsacle. In this case we compute the minimal distance of the range-finder.
            for i in range(width):
                # The weight is very simple, it is only an interval in the center of
                # the image to be shure to have enough place to move our robot.
                centering_weight = 1 if i < width / 4 or i > 3 * width / 4 else 3
                for j in range(height):
                    distance = RangeFinder.rangeImageGetDepth(image, width, i, j)
                    if distance > 100:  # distance may be infinity
                        distance = 100
                    if state == 'SEARCHING':
                        average_distance += distance * centering_weight
                    elif distance < minimal_distance:
                        minimal_distance = distance

            average_distance /= width * height
            if average_distance >= 1 and state == 'SEARCHING':
                state = 'MOVING'
            elif minimal_distance < 0.1 and state == 'MOVING':
                state = 'SEARCHING'
            if state == 'MOVING':
                left_speed = SPEED
                right_speed = SPEED
            else:
                left_speed = -SPEED
                right_speed = SPEED
            if save_image:
                # Save range-finder's current view as HDR image in home directory
                # compute the file path in the user directory
                if os.name == 'nt':
                    user_directory = os.environ['USERPROFILE']
                    # we need a DOS 8.3 path to support non-ascii characters in user name
                    BUFFER_SIZE = 1024
                    buffer = create_unicode_buffer(BUFFER_SIZE)
                    windll.kernel32.GetShortPathNameW(user_directory, buffer, BUFFER_SIZE)
                    user_directory = buffer.value
                else:
                    user_directory = os.environ['HOME']
                filename = os.path.join(user_directory, 'test.hdr')
                self.range_finder.saveImage(filename, 100)
                save_image = False

            # Set the motor speeds.
            self.left_motor.setVelocity(left_speed)
            self.right_motor.setVelocity(right_speed)


controller = Controller()
controller.run()
