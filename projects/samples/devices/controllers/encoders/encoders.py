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
An example of use of the encoders of a differential wheel
"""

from controller import Robot
import random


class Controller(Robot):
    timeStep = 64

    def __init__(self):
        super(Controller, self).__init__()

        # Get a handler to the motors and set target position to infinity (speed control).
        self.left_motor = self.getDevice('left wheel motor')
        self.right_motor = self.getDevice('right wheel motor')
        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))
        self.left_motor.setVelocity(0.0)
        self.right_motor.setVelocity(0.0)

        self.left_position_sensor = self.getDevice('left wheel sensor')
        self.right_position_sensor = self.getDevice('right wheel sensor')
        self.left_position_sensor.enable(self.timeStep)
        self.right_position_sensor.enable(self.timeStep)

    def run(self):
        SPEED_UNIT = 0.1
        ENCODER_UNIT = 0.25
        encoder_value = [0, 0]
        goal = [0, 0]
        speed = [0, 0]
        left_encoder_offset = 0.0
        right_encoder_offset = 0.0

        while self.step(self.timeStep) != -1:
            encoder_value[0] = int(ENCODER_UNIT * (self.left_position_sensor.getValue() - left_encoder_offset))
            encoder_value[1] = int(ENCODER_UNIT * (self.right_position_sensor.getValue() - right_encoder_offset))

            # simply turn the wheels in the direction corresponding to the
            # objectif position, when it is reached, stop the wheel.
            if encoder_value[0] != goal[0] or encoder_value[1] != goal[1]:
                for i in range(2):
                    speed[i] = goal[i] - encoder_value[i]
                    if speed[i] != 0:
                        speed[i] = 40 if speed[i] > 0 else -40

                # set the motor speeds
                self.left_motor.setVelocity(SPEED_UNIT * speed[0])
                self.right_motor.setVelocity(SPEED_UNIT * speed[1])
            else:
                # When both wheels are in place, we choose randomly new objectif
                # positions and we reset the encoders.
                for i in range(2):
                    goal[i] = random.randint(-10, 10)

                left_encoder_offset = self.left_position_sensor.getValue()
                right_encoder_offset = self.right_position_sensor.getValue()

                print(f'Goal position for the encoders: {goal[0]} {goal[1]}')


controller = Controller()
controller.run()
