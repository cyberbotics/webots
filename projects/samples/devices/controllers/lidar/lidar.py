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
Simulation of a lidar.
"""

from controller import Robot


class Controller(Robot):
    def __init__(self):
        super(Controller, self).__init__()
        self.timeStep = 32
        self.lidar = self.getDevice('lidar')
        self.lidar.enable(self.timeStep)
        self.lidar.enablePointCloud()
        self.us0 = self.getDevice('us0')
        self.us0.enable(self.timeStep)
        self.us1 = self.getDevice('us1')
        self.us1.enable(self.timeStep)
        self.left_motor = self.getDevice('left wheel motor')
        self.right_motor = self.getDevice('right wheel motor')
        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))
        self.left_motor.setVelocity(0)
        self.right_motor.setVelocity(0)

    def run(self):
        # empirical coefficients for collision avoidance
        coefficients = [[12.0, -6.0], [-10.0, 8.0]]
        base_speed = 6.0
        speed = [0, 0]
        us_value = [0, 0]
        while self.step(self.timeStep) != -1:
            us_value[0] = self.us0.getValue()
            us_value[1] = self.us1.getValue()
            for i in range(2):
                speed[i] = 0
                for k in range(2):
                    speed[i] += us_value[k] * coefficients[i][k]
            self.left_motor.setVelocity(base_speed + speed[0])
            self.right_motor.setVelocity(base_speed + speed[1])


controller = Controller()
controller.run()
