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
Wait 4 seconds before to slow down the motor linked with the wheel using the Brake node.
"""

from controller import Robot


class Controller(Robot):
    def __init__(self):
        super(Controller, self).__init__()
        self.timeStep = int(self.getBasicTimeStep())
        self.brake = self.getDevice('brake')
        self.motor = self.getDevice('motor')
        self.linear_motor_a = self.getDevice('linear motor a')
        self.linear_motor_b = self.getDevice('linear motor b')
        self.motor.setTorque(0.1)

    def run(self):
        print('Start the wheel motor...')
        while self.step(self.timeStep) != -1:
            if self.getTime() == 4.0:
                # At four seconds, the movement of the linear motors is started.
                print('Brake activated!')
                self.linear_motor_a.setPosition(self.linear_motor_a.getMaxPosition())
                self.linear_motor_b.setPosition(self.linear_motor_b.getMaxPosition())
            elif self.getTime() > 4.13:
                # When the red blocks touch the wheel, the braking of the main engine is performed.
                self.brake.setDampingConstant(1.0)
                self.motor.setTorque(0.0)
                break
        while self.step(self.timeStep) != -1:
            # Wait until the simulation is complete.
            pass


controller = Controller()
controller.run()
