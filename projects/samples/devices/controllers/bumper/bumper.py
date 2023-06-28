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
An example of use of a bumper touch sensor device.
"""

from controller import Robot


class Controller(Robot):
    timeStep = 64

    def __init__(self):
        super(Controller, self).__init__()
        self.bumper = self.getDevice('bumper')
        self.bumper.enable(self.timeStep)
        self.left_motor = self.getDevice('left wheel motor')
        self.right_motor = self.getDevice('right wheel motor')
        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))
        self.left_motor.setVelocity(0)
        self.right_motor.setVelocity(0)

    def run(self):
        SPEED = 4
        movement_counter = 0
        while self.step(self.timeStep) != -1:
            # When the touch sensor has detected something we begin the avoidance
            # movement.
            if self.bumper.getValue() > 0:
                movement_counter = 15
            # We use the movement_counter to manage the movements of the robot. When the value
            # is 0 we move straight, then when there is another value this means that
            # we are avoiding an obstacle. For avoiding we first move backward for
            # some cycles and then we turn on ourself.
            if movement_counter == 0:
                left_speed = SPEED
                right_speed = SPEED
            elif movement_counter >= 7:
                left_speed = -SPEED
                right_speed = -SPEED
                movement_counter -= 1
            else:
                left_speed = -SPEED / 2
                right_speed = SPEED
                movement_counter -= 1

            # Set the motor speeds
            self.left_motor.setVelocity(left_speed)
            self.right_motor.setVelocity(right_speed)


controller = Controller()
controller.run()
