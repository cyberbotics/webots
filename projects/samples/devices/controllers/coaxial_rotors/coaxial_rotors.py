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
This controller shows how to use the Propeller node. The torque effects of the
two coaxial rotors cancel out and hence allows yaw stabilization. The green
helicopter in propeller.wbt also manages to stabilize its altitude by means of a
simple velocity control based on GPS measurement.
"""

from controller import Supervisor


class Controller(Supervisor):
    HELIX_VELOCITY = 100.0

    timeStep = 64

    def __init__(self):
        super(Controller, self).__init__()
        self.gps = self.getDevice('gps')
        self.gps.enable(self.timeStep)
        self.inertial_unit = self.getDevice('inertial unit')
        self.inertial_unit.enable(self.timeStep)
        self.upper_motor = self.getDevice('upper_motor')
        self.lower_motor = self.getDevice('lower_motor')
        self.tail_motor = self.getDevice('tail_motor')
        self.upper_motor.setPosition(float('inf'))
        self.lower_motor.setPosition(float('inf'))
        self.tail_motor.setPosition(float('inf'))
        velocity = self.HELIX_VELOCITY + 1.0
        self.upper_motor.setVelocity(-velocity)
        self.lower_motor.setVelocity(velocity)
        self.tail_motor.setVelocity(15.0)

    def run(self):
        TARGET_ALTITUDE = 2.0
        LABEL_X = 0.05
        LABEL_Y = 0.95
        GREEN = 0x008800

        while self.step(self.timeStep) != -1:
            altitude = self.gps.getValues()[2]
            yaw = self.inertial_unit.getRollPitchYaw()[2]
            self.setLabel(0, f'Altitude: {altitude:1.1f} m', LABEL_X, LABEL_Y, 0.07, GREEN)
            self.setLabel(1, f'Yaw: {yaw:1.1f} rad', LABEL_X, LABEL_Y - 0.03, 0.07, GREEN)
            ratio = 1.0 - altitude / TARGET_ALTITUDE
            velocity = self.HELIX_VELOCITY + ratio
            self.upper_motor.setVelocity(-velocity)
            self.lower_motor.setVelocity(velocity)


controller = Controller()
controller.run()
