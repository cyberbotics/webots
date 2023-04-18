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
An implementation of a GPS with a Supervisor equipped with an Emitter, playing
the role of the sattelite and a robot equipped with a Receiver, playing the role
of the GPS receiver. This file is the supervisor controller.
"""

from controller import Supervisor
# import struct


class Controller(Supervisor):
    SPEED = 6
    timeStep = 64

    def __init__(self):
        super(Controller, self).__init__()

        self.emitter = self.getDevice('emitter')
        self.robot_translation = self.getFromDef('GPS_ROBOT').getField('translation')

    def run(self):
        while self.step(self.timeStep) != -1:
            # At each step, the position of the robot is get and sent through the emitter
            self.emitter.send(self.robot_translation.getSFVec3f())


controller = Controller()
controller.run()
