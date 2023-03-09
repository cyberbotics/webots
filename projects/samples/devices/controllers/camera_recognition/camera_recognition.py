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
An example of use of a camera device with recognition capability.
"""

from controller import Robot


class Controller(Robot):
    def __init__(self):
        super(Controller, self).__init__()
        self.timeStep = 64
        self.camera = self.getDevice('camera')
        self.camera.enable(self.timeStep)
        self.camera.recognitionEnable(self.timeStep)
        self.left_motor = self.getDevice('left wheel motor')
        self.right_motor = self.getDevice('right wheel motor')
        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))
        self.left_motor.setVelocity(-1.5)
        self.right_motor.setVelocity(1.5)

    def run(self):
        while self.step(self.timeStep) != -1:
            # Get current number of object recognized
            number_of_objects = self.camera.getRecognitionNumberOfObjects()
            print(f'Recognized {number_of_objects} objects.')
            print(' ')
            objects = self.camera.getRecognitionObjects()
            counter = 1
            for object in objects:
                position = object.getPosition()
                orientation = object.getOrientation()
                size = object.getSize()
                position_on_image = object.getPositionOnImage()
                size_on_image = object.getSizeOnImage()
                number_of_colors = object.getNumberOfColors()
                colors = object.getColors()
                print(f' Object {counter}/{number_of_objects}: {object.getModel()} (id = {object.getId()})')
                print(f' Position: {position[0]} {position[1]} {position[2]}')
                print(f' Orientation: {object.orientation[0]} {orientation[1]} {orientation[2]} {orientation[3]}')
                print(f' Size: {size[0]} x {size[1]}')
                print(f' Position on camera image: {position_on_image[0]} {position_on_image[1]}')
                print(f' Size on camera image: {size_on_image[0]} x {size_on_image[1]}')
                for j in range(number_of_colors):
                    print(f'  Color {j + 1}/{number_of_colors}: '
                          f'{colors[3 * j]} {colors[3 * j + 1]} {colors[3 * j + 2]}')
                print(' ')
                counter += 1


controller = Controller()
controller.run()
