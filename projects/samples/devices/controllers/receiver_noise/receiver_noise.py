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
Example usage of signalStrengthNoise and directionNoise in a Receiver,
used to compute the position of an Emitter and compared to the actual
position of the Emitter measured with a noise-free GPS.
"""

from controller import Robot, AnsiCodes
import math


class Controller(Robot):
    SPEED = 6
    COMMUNICATION_CHANNEL = 1
    timeStep = 64

    def __init__(self):
        super(Controller, self).__init__()

        # As we are using the same controller for the emitter and the receiver,
        # we need to differentiate them
        if self.getName() == 'MyBot emitter':
            self.robot_type = 'EMITTER'
            self.communication = self.getDevice('emitter')
            # If the channel is not the right one, change it
            channel = self.communication.getChannel()
            if channel != self.COMMUNICATION_CHANNEL:
                self.communication.setChannel(self.COMMUNICATION_CHANNEL)
            self.gps = self.getDevice('gps')
            self.gps.enable(self.timeStep)

        elif self.getName() == 'MyBot receiver':
            self.robot_type = 'RECEIVER'
            self.communication = self.getDevice('receiver')
            self.communication.enable(self.timeStep)
        else:
            print(f'Unrecognized robot name {self.getName()}. Exiting...')
            return

        self.ds0 = self.getDevice('ds0')
        self.ds1 = self.getDevice('ds1')
        self.ds0.enable(self.timeStep)
        self.ds1.enable(self.timeStep)

        # Get a handler to the motors and set target position to infinity (speed control).
        self.left_motor = self.getDevice('left wheel motor')
        self.right_motor = self.getDevice('right wheel motor')
        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))
        self.left_motor.setVelocity(0.0)
        self.right_motor.setVelocity(0.0)

    def run(self):
        def clamp(n, smallest, largest):
            return max(smallest, min(n, largest))

        while self.step(self.timeStep) != -1:
            # The emitter simply sends the message but the receiver
            # has to check if there is something before it can reads the buffer.
            if self.robot_type == 'EMITTER':
                print('EMITTER')
                self.communication.send('Hello!')
                gps_position = self.gps.getValues()
                # print real position measured from the GPS
                print(AnsiCodes.CLEAR_SCREEN)
                # print(f'GPS position:     time = {self.getTime():%.3f}   X = {gps_position[0]:.3f} Y = {gps_position[1]:.3f}')
                print(f'GPS position: time = {self.getTime():.3f}   ' +
                      f'X = {gps_position[0]:.3f} Y = {gps_position[1]:.3f}')
            else:
                # is there at least one packet in the receiver's queue?
                if self.communication.getQueueLength() > 0:
                    # compute and print position of the Emitter from signal strength and direction */
                    signal_strength = self.communication.getSignalStrength()
                    direction = self.communication.getEmitterDirection()
                    distance = 1 / math.sqrt(signal_strength)
                    print(AnsiCodes.CLEAR_SCREEN)
                    print(f'Emitter position: time = {self.getTime():.3f}   ' +
                          f'X = {direction[0] * distance:.3f} Y = {direction[1] * distance:.3f}')
                    self.communication.nextPacket()

            if self.robot_type == 'EMITTER':
                ds0_value = self.ds0.getValue()
                ds1_value = self.ds1.getValue()
                if ds1_value > 500:
                    # If both distance sensors are detecting something, this means that
                    # we are facing a wall. In this case we need to move backwards.
                    if ds0_value > 200:
                        left_speed = -self.SPEED / 2
                        right_speed = -self.SPEED
                    else:
                        # we turn proportionally to the sensors value because the
                        # closer we are from the wall, the more we need to turn.
                        left_speed = -ds1_value / 100
                        right_speed = ds0_value / 100 + 0.5
                elif ds0_value > 500:
                    left_speed = ds1_value / 100 + 0.5
                    right_speed = -ds0_value / 100
                else:
                    # if nothing was detected we can move forward at maximal speed.
                    left_speed = self.SPEED
                    right_speed = self.SPEED

                # set the motor speeds.
                self.left_motor.setVelocity(clamp(left_speed, -10.0, 10.0))
                self.right_motor.setVelocity(clamp(right_speed, -10.0, 10.0))


controller = Controller()
controller.run()
