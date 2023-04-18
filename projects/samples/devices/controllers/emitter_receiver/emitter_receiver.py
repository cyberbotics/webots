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
A controller which sends data from the emitter to the receiver while the robots avoid the obstacles.
"""

from controller import Robot


class Controller(Robot):
    SPEED = 6
    COMMUNICATION_CHANNEL = 1
    timeStep = 64

    def __init__(self):
        super(Controller, self).__init__()

        # Get a handler to the motors and set target position to infinity (speed control).
        self.left_motor = self.getDevice('left wheel motor')
        self.right_motor = self.getDevice('right wheel motor')
        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))

        # As we are using the same controller for the emitter and the receiver,
        # we need to differentiate them
        if self.getName() == 'MyBot emitter':
            self.robot_type = 'EMITTER'
            self.communication = self.getDevice('emitter')
            # If the channel is not the right one, change it
            channel = self.communication.getChannel()
            if channel != self.COMMUNICATION_CHANNEL:
                self.communication.setChannel(self.COMMUNICATION_CHANNEL)
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

    def run(self):
        message_printed = 0
        while self.step(self.timeStep) != -1:
            # The emitter simply sends the message but the receiver
            # has to check if there is something before it can reads the buffer.
            if self.robot_type == 'EMITTER':
                self.communication.send("Hello!")
            else:
                # is there at least one packet in the receiver's queue?
                if self.communication.getQueueLength() > 0:
                    # read current packet's data
                    buffer = self.communication.getString()
                    if message_printed != 1:
                        print(f'Communicating: received "{buffer}"')
                        message_printed = 1
                    self.communication.nextPacket()  # fetch next packet
                elif message_printed != 2:
                    print('Communication broken!')
                    message_printed = 2

            ds0_value = self.ds0.getValue()
            ds1_value = self.ds1.getValue()
            if ds1_value > 500:
                # If both distance sensors are detecting something, this means that
                # we are facing a wall. In this case we need to move backwards.
                if ds0_value > 200:
                    if self.robot_type == 'EMITTER':
                        left_speed = -self.SPEED
                        right_speed = -self.SPEED / 2
                    else:
                        left_speed = -self.SPEED / 2
                        right_speed = -self.SPEED
                else:
                    # we turn proportionnaly to the sensors value because the
                    # closer we are from the wall, the more we need to turn.
                    left_speed = -ds1_value / 100
                    right_speed = (ds0_value / 100) + 0.5
            elif ds0_value > 500:
                left_speed = (ds1_value / 100) + 0.5
                right_speed = -ds0_value / 100
            else:  # if nothing was detected we can move forward at maximal speed.
                left_speed = self.SPEED
                right_speed = self.SPEED

            self.left_motor.setVelocity(left_speed)
            self.right_motor.setVelocity(right_speed)


controller = Controller()
controller.run()
