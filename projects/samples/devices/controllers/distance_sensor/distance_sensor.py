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
An example of use of the distance sensors.
"""

from controller import Robot


class Controller(Robot):
    NB_SENSORS = 8
    RANGE = 512
    MATRIX = [[11, 12, 8, -2, -3, -5, -7, -9], [-9, -8, -5, -1, -2, 6, 12, 11]]
    timeStep = 64

    def __init__(self):
        super(Controller, self).__init__()
        self.ps = []
        for i in range(self.NB_SENSORS):
            self.ps.append(self.getDevice('ds' + str(i)))
            self.ps[i].enable(self.timeStep)

        self.left_motor = self.getDevice('left wheel motor')
        self.right_motor = self.getDevice('right wheel motor')
        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))
        self.left_motor.setVelocity(0.5)
        self.right_motor.setVelocity(-0.5)

    def run(self):
        while self.step(self.timeStep) != -1:
            sensor_value = []
            for i in range(self.NB_SENSORS):
                sensor_value.append(self.ps[i].getValue())
            speed = [0, 0]
            for i in range(2):
                for j in range(self.NB_SENSORS):
                    # We need to recenter the value of the sensor to be able to get
                    # negative values too. This will allow the wheels to go  backward too.
                    speed[i] += self.MATRIX[i][j] * (1 - (sensor_value[j] / self.RANGE))

            # Set the motor speeds
            self.left_motor.setVelocity(0.2 * speed[0])
            self.right_motor.setVelocity(0.2 * speed[1])


controller = Controller()
controller.run()
