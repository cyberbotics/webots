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
An example of the use of the battery of a robot
"""

from controller import Robot, AnsiCodes


class Controller(Robot):
    timeStep = 64

    def __init__(self):
        super(Controller, self).__init__()

        self.ds0 = self.getDevice('ds0')
        self.ds1 = self.getDevice('ds1')
        self.ds0.enable(self.timeStep)
        self.ds1.enable(self.timeStep)
        self.batterySensorEnable(self.timeStep)

        # Get a handler to the motors and set target position to infinity (speed control).
        self.left_motor = self.getDevice('left wheel motor')
        self.right_motor = self.getDevice('right wheel motor')
        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))
        self.left_motor.setVelocity(0.0)
        self.right_motor.setVelocity(0.0)

    def run(self):
        def clamp(n, smallest, largest):
            return max(smallest, min(n, largest))

        SPEED = 6
        max_speed = self.left_motor.getMaxVelocity()
        while self.step(self.timeStep) != -1:
            print(AnsiCodes.CLEAR_SCREEN)
            print(f'Battery sensor value: {self.batterySensorGetValue()} J')
            ds0_value = self.ds0.getValue()
            ds1_value = self.ds1.getValue()

            if ds1_value > 500:
                # If both distance sensors are detecting something, this means that
                # we are facing a wall. In this case we need to move backwards.
                if ds0_value > 200:
                    left_speed = -SPEED
                    right_speed = -SPEED / 2
                else:
                    # We turn proportionally to the sensors value because the
                    # closer we are from the wall, the more we need to turn.
                    left_speed = -ds1_value / 50
                    right_speed = ds0_value / 50
                    left_speed = clamp(left_speed, -max_speed, max_speed)
                    right_speed = clamp(right_speed, -max_speed, max_speed)
            elif ds0_value > 500:
                left_speed = ds1_value / 50
                right_speed = -ds0_value / 50
                left_speed = clamp(left_speed, -max_speed, max_speed)
                right_speed = clamp(right_speed, -max_speed, max_speed)
            else:
                # If nothing was detected we can move forward at maximal speed.
                left_speed = SPEED
                right_speed = SPEED

            self.left_motor.setVelocity(left_speed)
            self.right_motor.setVelocity(right_speed)


controller = Controller()
controller.run()
