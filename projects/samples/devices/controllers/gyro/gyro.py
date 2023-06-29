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
Temo/Test for the Gyro device.
A three-axis Gyro device is mounted on three orthogonal and motorized rotation axes.
The demo activates the motor axes one after the other and prints the gyro output.
The sensor output should correctly reflects the currently active motor/rotation axis.
The red, green and blue cylinders are aligned with the Gyro's x, y and z axes respectively.
Note that, the outputs climbs up to 10 rad/s because this is the maximum rotation speed
(maxVelocity) of each motor.
"""

import sys
from controller import Robot, AnsiCodes


class Controller(Robot):
    def __init__(self):
        super(Controller, self).__init__()
        self.timeStep = 8
        self.gyro = self.getDevice('gyro')
        self.gyro.enable(self.timeStep)
        self.motor_x = self.getDevice('motor_x')
        self.motor_y = self.getDevice('motor_y')
        self.motor_z = self.getDevice('motor_z')

    def steps(self, motor):
        motor.setPosition(62.83)
        for t in range(0, 10000, self.timeStep):
            if self.step(self.timeStep) == -1:
                sys.exit()
            vel = self.gyro.getValues()
            print(AnsiCodes.CLEAR_SCREEN)
            print(f'rotation axes: [ x y z ] = [ {vel[0]:+.2f} {vel[1]:+.2f} {vel[2]:+.2f} ]')

    def run(self):
        self.steps(self.motor_x)
        self.steps(self.motor_y)
        self.steps(self.motor_z)


controller = Controller()
controller.run()
