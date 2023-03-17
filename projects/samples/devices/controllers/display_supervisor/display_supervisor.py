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
This supervisor tracks the absolute position of the robot and displays the result on a Display
device.
"""

from controller import Supervisor
import os
if os.name == 'nt':
    from ctypes import create_unicode_buffer, windll


class Controller(Supervisor):
    def __init__(self):
        super(Controller, self).__init__()
        self.timeStep = 64
        self.ground_display = self.getDevice('ground_display')

    def run(self):
        GROUND_X = 1.0
        GROUND_Y = 1.0
        LIGHT_GRAY = 0x505050
        RED = 0xBB2222
        GREEN = 0x22BB11
        BLUE = 0x2222BB

        width = self.ground_display.getWidth()
        height = self.ground_display.getHeight()

        mybot = self.getFromDef('MYBOT')
        translation_field = mybot.getField('translation')
        self.ground_display.setColor(LIGHT_GRAY)
        self.ground_display.fillRectangle(0, 0, width, height)
        self.ground_display.setColor(RED)
        self.ground_display.drawLine(width / 2, 0, width / 2, height - 1)
        self.ground_display.drawText('x', width / 2 - 10, height - 10)
        self.ground_display.setColor(GREEN)
        self.ground_display.drawLine(0, height / 2, width - 1, height / 2)
        self.ground_display.drawText('y', width - 10, height / 2 - 10)

        to_store = None  # image to save as a file
        counter = 0      # time steps counter
        while self.step(self.timeStep) != -1:
            translation = translation_field.getSFVec3f()
            counter += 1

            # display the robot position
            self.ground_display.setOpacity(0.03)
            self.ground_display.setColor(BLUE)
            self.ground_display.fillOval(width - width * (translation[1] + GROUND_X / 2) / GROUND_X,
                                         height - height * (translation[0] + GROUND_Y / 2) / GROUND_Y,
                                         4, 4)
            # Clear previous to_store
            if to_store is not None:
                self.ground_display.imageDelete(to_store)
                to_store = None

            # Every 50 steps, store the resulted image into a file
            if counter % 50 == 0:
                to_store = self.ground_display.imageCopy(0, 0, width, height)
                # compute the path to store the image in user directory
                if os.name == 'nt':
                    user_directory = os.environ['USERPROFILE']
                    # we need a DOS 8.3 path to support non-ascii characters in user name
                    BUFFER_SIZE = 1024
                    buffer = create_unicode_buffer(BUFFER_SIZE)
                    windll.kernel32.GetShortPathNameW(user_directory, buffer, BUFFER_SIZE)
                    user_directory = buffer.value
                else:
                    user_directory = os.environ['HOME']
                filename = os.path.join(user_directory, 'screenshot.png')
                self.ground_display.imageSave(to_store, filename)


controller = Controller()
controller.run()
