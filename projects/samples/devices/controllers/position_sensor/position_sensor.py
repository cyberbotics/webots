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
An example of a controller using the position sensor. The system represents
and inverted pendulum consisting of a pole conected to a robot by an hinge
joint. The speed of the robot is adjusted based on the position of the pole
in order to balance it.
"""

from controller import Robot, AnsiCodes
import math


class Controller(Robot):
    def __init__(self):
        super(Controller, self).__init__()
        self.timeStep = int(self.getBasicTimeStep())

        self.position_sensor = self.getDevice('position sensor')
        self.position_sensor.enable(self.timeStep)

        # Get a handler to the motors and set target position to infinity (speed control).
        self.left_motor = self.getDevice('left wheel motor')
        self.right_motor = self.getDevice('right wheel motor')
        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))
        self.left_motor.setVelocity(0.0)
        self.right_motor.setVelocity(0.0)

    def run(self):
        KP = 35
        KI = 5
        KD = 28.9
        previous_position = 0
        integral = 0.0

        while self.step(self.timeStep) != -1:
            position = self.position_sensor.getValue()
            if math.fabs(position) > 0.7:  # pole has fallen
                break
            # PID control
            integral += (position + previous_position) * 0.5
            derivative = position - previous_position
            speed = KP * position + KI * integral + KD * derivative
            # check maximum speed
            if speed > 100:
                speed = 100
            elif speed < -100:
                speed = -100
            self.left_motor.setVelocity(-speed)
            self.right_motor.setVelocity(-speed)
            print(AnsiCodes.CLEAR_SCREEN)
            print(f'Position: {position:+.4f} -> control force: {speed:+.4f}')
            previous_position = position

        print('Pole has fallen.')
        self.left_motor.setVelocity(0)
        self.right_motor.setVelocity(0)
        self.step(self.timeStep)


controller = Controller()
controller.run()
