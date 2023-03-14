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
An example of use of a speaker device.
"""

from controller import Robot, Speaker


class Controller(Robot):
    timeStep = 64

    def __init__(self):
        super(Controller, self).__init__()
        self.speaker = self.getDevice('speaker')
        self.left_motor = self.getDevice('left wheel motor')
        self.right_motor = self.getDevice('right wheel motor')
        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))
        # follow a circle around the center of the arena
        self.left_motor.setVelocity(8)
        self.right_motor.setVelocity(10)

    def run(self):
        # use the speaker to play a sound file
        Speaker.playSound(self.speaker, self.speaker, 'sounds/robot_sound.wav', 1.0, 1.0, 0.0, True)
        print('I am going to use the speaker to play a sound file while I am moving in circle around the center of the arena.')
        print('Please make sure the sound is enabled and do not move the viewpoint.')

        offset = self.getTime()
        while self.step(self.timeStep) != -1:
            # every 5 seconds play a second sound file using the same speaker
            if self.getTime() - offset > 5.0:
                Speaker.playSound(self.Speaker, self.speaker, 'sounds/robot_bip.wav', 0.7, 1.0, 0.0, False)
            offset = self.getTime()


controller = Controller()
controller.run()
