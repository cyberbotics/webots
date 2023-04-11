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
Turn the robot using its motors.
Switch on the bottommost LED using the gravity force given by the Accelerometer feedback.
"""

from controller import Robot


class Controller(Robot):
    def __init__(self):
        super(Controller, self).__init__()
        self.timeStep = int(self.getBasicTimeStep())
        self.accelerometer = self.getDevice('accelerometer')
        self.accelerometer.enable(self.timeStep)
        self.front_led = self.getDevice('front led')
        self.back_led = self.getDevice('back led')
        self.left_led = self.getDevice('left led')
        self.right_led = self.getDevice('right led')
        self.left_motor = self.getDevice('left wheel motor')
        self.right_motor = self.getDevice('right wheel motor')
        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))
        self.left_motor.setVelocity(0.5)
        self.right_motor.setVelocity(-0.5)

    def run(self):
        while self.step(self.timeStep) != -1:
            acceleration = self.accelerometer.getValues()
            if abs(acceleration[1]) > abs(acceleration[0]):
                self.front_led.set(False)
                self.back_led.set(False)
                self.left_led.set(acceleration[1] > 0.0)
                self.right_led.set(acceleration[1] < 0.0)
            else:
                self.front_led.set(acceleration[0] < 0.0)
                self.back_led.set(acceleration[0] > 0.0)
                self.left_led.set(False)
                self.right_led.set(False)


controller = Controller()
controller.run()
