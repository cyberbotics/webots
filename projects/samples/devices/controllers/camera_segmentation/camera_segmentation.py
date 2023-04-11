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

from controller import Robot, Display


class Controller(Robot):
    def __init__(self):
        super(Controller, self).__init__()
        self.timeStep = 64
        self.camera = self.getDevice('camera')
        self.camera.enable(self.timeStep)
        self.camera.recognitionEnable(self.timeStep)
        self.camera.enableRecognitionSegmentation()
        self.display = self.getDevice('segmented image display')
        self.left_motor = self.getDevice('left wheel motor')
        self.right_motor = self.getDevice('right wheel motor')
        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))
        self.left_motor.setVelocity(-1.5)
        self.right_motor.setVelocity(1.5)

    def run(self):
        width = self.camera.getWidth()
        height = self.camera.getHeight()
        while self.step(self.timeStep) != -1:
            if self.camera.isRecognitionSegmentationEnabled() and self.camera.getRecognitionSamplingPeriod() > 0:
                # Get the segmented image and display it in the Display
                data = self.camera.getRecognitionSegmentationImage()
                if data:
                    segmented_image = self.display.imageNew(data, Display.BGRA, width, height)
                    self.display.imagePaste(segmented_image, 0, 0, False)
                    self.display.imageDelete(segmented_image)


controller = Controller()
controller.run()
