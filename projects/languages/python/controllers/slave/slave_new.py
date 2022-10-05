# Copyright 1996-2022 Cyberbotics Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
This controller gives to its robot the following behavior:
According to the messages it receives, the robot change its
behavior.
"""

from webots import AnsiCodes, Camera, DistanceSensor, Motor, Receiver, Robot
from common import common_print


class Enumerate(object):
    def __init__(self, names):
        for number, name in enumerate(names.split()):
            setattr(self, name, number)


class Slave (Robot):

    Mode = Enumerate('STOP MOVE_FORWARD AVOID_OBSTACLES TURN')
    max_speed = 10.0
    mode = Mode.AVOID_OBSTACLES
    motors = []
    distance_sensors = []

    def bound_speed(self, speed):
        return max(-self.max_speed, min(self.max_speed, speed))

    def __init__(self):
        super().__init__()
        self.mode = self.Mode.AVOID_OBSTACLES
        self.camera = Camera('camera', sampling_period=4 * int(self.basic_time_step))
        self.receiver = Receiver('receiver')
        self.motors.append(Motor('left wheel motor'))
        self.motors.append(Motor('right wheel motor'))
        self.motors[0].target_position = float('inf')
        self.motors[1].target_position = float("inf")
        self.motors[0].target_velocity = 0.0
        self.motors[1].target_velocity = 0.0
        for number in range(0, 2):
            self.distance_sensors.append(DistanceSensor('ds' + str(number)))

    def run(self):
        while True:
            # Read the supervisor order.
            if self.receiver.queue_length > 0:
                message = self.receiver.string
                self.receiver.next_packet()
                print('I should ' + AnsiCodes.RED_FOREGROUND + message + AnsiCodes.RESET + '!')
                if message == 'avoid obstacles':
                    self.mode = self.Mode.AVOID_OBSTACLES
                elif message == 'move forward':
                    self.mode = self.Mode.MOVE_FORWARD
                elif message == 'stop':
                    self.mode = self.Mode.STOP
                elif message == 'turn':
                    self.mode = self.Mode.TURN
            delta = self.distance_sensors[0].value - self.distance_sensors[1].value
            speeds = [0.0, 0.0]

            # Send actuators commands according to the mode.
            if self.mode == self.Mode.AVOID_OBSTACLES:
                speeds[0] = self.bound_speed(self.max_speed / 2 + 0.1 * delta)
                speeds[1] = self.bound_speed(self.max_speed / 2 - 0.1 * delta)
            elif self.mode == self.Mode.MOVE_FORWARD:
                speeds[0] = self.max_speed
                speeds[1] = self.max_speed
            elif self.mode == self.Mode.TURN:
                speeds[0] = self.max_speed / 2
                speeds[1] = -self.max_speed / 2
            self.motors[0].target_velocity = speeds[0]
            self.motors[1].target_velocity = speeds[1]

            # Perform a simulation step, quit the loop when
            # Webots is about to quit.
            if self.step() == -1:
                break


controller = Slave()
common_print('slave')
controller.run()
