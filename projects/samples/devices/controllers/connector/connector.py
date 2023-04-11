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
An example of controller using a Connector device.
"""

from controller import Robot
import math


class Controller(Robot):
    SPEED = 2
    timeStep = 64

    def __init__(self):
        super(Controller, self).__init__()
        self.connector = self.getDevice('connector')
        self.connector.enablePresence(self.timeStep)
        self.upper_motor = self.getDevice('upper motor')
        self.upper_motor.setPosition(-1.57)
        self.upper_position_sensor = self.getDevice('upper sensor')
        self.upper_position_sensor.enable(self.timeStep)
        self.left_motor = self.getDevice('left wheel motor')
        self.right_motor = self.getDevice('right wheel motor')
        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))
        self.left_motor.setVelocity(0.0)
        self.right_motor.setVelocity(0.0)

    def run(self):
        robot_number = int(self.getName()[6])
        hop_counts = 0
        direction = 1
        state = 'CONNECTOR_IN_PLACE'
        while self.step(self.timeStep) != -1:
            left_speed = 0
            right_speed = 0
            if state == 'CONNECTOR_IN_PLACE':  # wait for the motor to be in place
                if math.fabs(self.upper_position_sensor.getValue() + (direction * 1.56)) < 0.01:
                    state = 'CONNECTING'
            elif state == 'CONNECTING':  # wait for another Connector device to lock with
                if self.connector.getPresence() == 1:
                    self.connector.lock()
                    self.upper_motor.setPosition(direction * 1.57)
                    state = 'PASSING_OVER'
                # if it is the robot which is moving, we need to take it closer to
                # the other one so that they can attach to each other
                if robot_number == 1:
                    left_speed = direction * self.SPEED
                    right_speed = direction * self.SPEED
            elif state == 'PASSING_OVER':  # wait for the motor to be in the new place (hopping)
                if math.fabs(self.upper_position_sensor.getValue() - (direction * 1.56)) < 0.01:
                    self.connector.unlock()
                    state = 'NEXT_HOP'
                    hop_counts += 1
            elif state == 'NEXT_HOP':  # wait for the other Connector device to be far enough
                if self.connector.getPresence() == 0:
                    # if it is the robot which is moving, we need to replace the
                    # motor in order to be able to atach to the next robot
                    if robot_number == 1:
                        if hop_counts % 2 == 0:
                            direction *= -1
                        self.upper_motor.setPosition(-1.57 * direction)
                        state = 'CONNECTOR_IN_PLACE'

                    # If not, the motor is already in the right place as the moving
                    # robot will come back at the same place.
                    else:
                        state = 'CONNECTING'
                        direction *= -1
                # the moving robot needs to get away from the connector in order to
                # avoid attaching always to the same robot.
                if robot_number == 1:
                    left_speed = direction * self.SPEED
                    right_speed = direction * self.SPEED
            # set the motor speeds
            self.left_motor.setVelocity(left_speed)
            self.right_motor.setVelocity(right_speed)


controller = Controller()
controller.run()
