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
An example of use of a camera focus device.
"""

from controller import Robot


class Controller(Robot):
    def __init__(self):
        super(Controller, self).__init__()
        self.timeStep = 32
        self.camera = self.getDevice('camera')
        self.camera.enable(self.timeStep)
        self.distance_sensor = self.getDevice('distance sensor')
        self.distance_sensor.enable(self.timeStep)
        self.left_motor = self.getDevice('left wheel motor')
        self.right_motor = self.getDevice('right wheel motor')
        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))
        self.left_motor.setVelocity(-1)
        self.right_motor.setVelocity(1)

    def run(self):
        while self.step(self.timeStep) != -1:
            object_distance = self.distance_sensor.getValue() / 1000
            self.camera.setFocalDistance(object_distance)


controller = Controller()
controller.run()
