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
The axis of a gear train lags behind the rotor that drives it
"""

from controller import Robot


class Controller(Robot):
    timeStep = 16

    def __init__(self):
        super(Controller, self).__init__()

        # Get a handler to the motors and set target position to infinity (speed control).
        self.rotor_motor = self.getDevice('rotor motor')
        self.rotor_motor.setPosition(float('inf'))
        self.rotor_motor.setVelocity(0.2)

        self.rotor_sensor = self.getDevice('rotor sensor')
        self.rotor_sensor.enable(self.timeStep)

    def run(self):
        while self.step(self.timeStep) != -1:
            position = self.rotor_sensor.getValue()
            if position > 1.0471:
                self.rotor_motor.setVelocity(-0.2)
            elif position < -1.0471:
                self.rotor_motor.setVelocity(0.2)


controller = Controller()
controller.run()
