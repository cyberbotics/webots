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
This controller gives to its node the following behavior:
Listen the keyboard. According to the pressed key, send a
message through an emitter or handle the position of Robot1.
"""

from webots import Supervisor, Emitter, Keyboard, Node, Field
from common import common_print


class Driver (Supervisor):
    time_step = 128

    def __init__(self):
        super().__init__()
        self.emitter = Emitter('emitter')
        self.translation = Field(Node('ROBOT1'), 'translation')
        self.keyboard = Keyboard()

    def run(self):
        self.display_help()
        previous_message = ''

        # Main loop.
        while True:
            # Deal with the pressed keyboard key.
            k = self.keyboard.get_key()
            message = ''
            if k == 'A':
                message = 'avoid obstacles'
            elif k == 'F':
                message = 'move forward'
            elif k == 'S':
                message = 'stop'
            elif k == 'T':
                message = 'turn'
            elif k == 'I':
                self.display_help()
            elif k == 'G':
                t = self.translation.value
                print('ROBOT1 is located at (' + str(t[0]) + ',' + str(t[2]) + ')')
            elif k == 'R':
                print('Teleport ROBOT1 at (0.1, 0.3, 0.0)')
                self.translation.value = [0.1, 0.3, 0.0]

            # Send a new message through the emitter device.
            if message != '' and message != previous_message:
                previous_message = message
                print('Please, ' + message)
                self.emitter.send(message)

            # Perform a simulation step, quit the loop when
            # Webots is about to quit.
            if self.step(self.time_step) == -1:
                break

    def display_help(self):
        print(
            'Commands:\n'
            ' I for displaying the commands\n'
            ' A for avoid obstacles\n'
            ' F for move forward\n'
            ' S for stop\n'
            ' T for turn\n'
            ' R for positioning ROBOT1 at (-0.3,-0.1)\n'
            ' G for knowing the (x,y) position of ROBOT1'
        )


controller = Driver()
common_print('driver')
controller.run()
