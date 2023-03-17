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
This controller shows how to use the Propeller node. The torque effect generated
by the rotor makes the blue helicopter in propeller.wbt spin and eventually fall
in spite of control commands aiming at at yaw and altitude stabilization.
"""

from controller import Supervisor


class Controller(Supervisor):
    MAIN_HELIX_VELOCITY = 100.0
    TAIL_HELIX_VELOCITY = 100.0

    timeStep = 64

    def __init__(self):
        super(Controller, self).__init__()
        self.gps = self.getDevice('gps')
        self.gps.enable(self.timeStep)
        self.inertial_unit = self.getDevice('inertial unit')
        self.inertial_unit.enable(self.timeStep)
        self.motor = self.getDevice('motor')
        self.motor.setPosition(float('inf'))
        self.motor.setVelocity(self.MAIN_HELIX_VELOCITY + 1)

    def run(self):
        TARGET_ALTITUDE = 2.0
        LABEL_X = 0.05
        LABEL_Y = 0.02
        BLUE = 0x0000FF

        while self.step(self.timeStep) != -1:
            altitude = self.gps.getValues()[2]
            yaw = self.inertial_unit.getRollPitchYaw()[2]
            self.setLabel(3, f'Altitude: {altitude:1.1f} m', LABEL_X, LABEL_Y, 0.07, BLUE)
            self.setLabel(2, f'Yaw: {yaw:1.1f} rad', LABEL_X, LABEL_Y + 0.03, 0.07, BLUE)
            ratio = 1.0 - altitude / TARGET_ALTITUDE
            self.motor.setVelocity(self.MAIN_HELIX_VELOCITY + ratio)


controller = Controller()
controller.run()
