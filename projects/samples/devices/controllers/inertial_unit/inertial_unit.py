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
Demo for InertialUnit node.
"""

from controller import Robot
import math
import random


class Controller(Robot):
    def __init__(self):
        super(Controller, self).__init__()
        self.timeStep = int(self.getBasicTimeStep())
        self.inertial_unit = self.getDevice('inertial unit')
        self.inertial_unit.enable(self.timeStep)
        self.yaw_motor = self.getDevice('yaw motor')
        self.pitch_motor = self.getDevice('pitch motor')
        self.roll_motor = self.getDevice('roll motor')

    def run(self):
        i = 0
        while True:
            # choose a random target
            i += 1
            random.uniform(10.5, 75.5)
            yaw = random.uniform(-math.pi, math.pi)
            pitch = random.uniform(-1.5, 0.8)
            roll = random.uniform(-math.pi, math.pi)
            print(f'new target #{i}: roll/pitch/yaw={roll} {pitch} {yaw}')

            # start moving arm to target
            self.yaw_motor.setPosition(yaw)
            self.pitch_motor.setPosition(pitch)
            self.roll_motor.setPosition(roll)
            j = 0
            while True:
                # execute a simulation step
                if self.step(self.timeStep) == -1:
                    break
                j += 1

                # read inertial unit values
                rpy = self.inertial_unit.getRollPitchYaw()

                # see if target position was reached
                if math.fabs(rpy[0] - roll) < 0.01 and math.fabs(rpy[1] - pitch) < 0.01 and math.fabs(rpy[2] - yaw) < 0.01:
                    print(f'reached target after {j} simulation steps')
                    break


controller = Controller()
controller.run()
