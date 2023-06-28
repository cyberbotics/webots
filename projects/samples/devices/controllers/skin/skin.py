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
Simple example of skin animation.
"""

from controller import Robot
import sys


class Controller(Robot):
    def __init__(self):
        super(Controller, self).__init__()
        self.timeStep = int(self.getBasicTimeStep())
        self.skin = self.getDevice('skin')

    def run(self):
        bones_count = self.skin.getBoneCount()
        print(f'The skin model is made of {bones_count} bones:')
        leg_bone_index = -1
        for i in range(bones_count):
            name = self.skin.getBoneName(i)
            print(f'  Bone {i}: {name}')
            if name == 'LeftLeg':
                leg_bone_index = i

        if leg_bone_index == -1:
            print('"LeftLeg" bone not found.', file=sys.stderr)
            sys.exit()

        # get root 'Hips' bone position
        absolute_hips_position = self.skin.getBonePosition(0, True)
        hips_position = [absolute_hips_position[0],
                         absolute_hips_position[1],
                         absolute_hips_position[2]]

        # get initial right leg orientation
        relative_leg_orientation = self.skin.getBoneOrientation(leg_bone_index, False)
        leg_orientation = [relative_leg_orientation[0],
                           relative_leg_orientation[1],
                           relative_leg_orientation[2],
                           relative_leg_orientation[3]]

        print('Move "LeftLeg" bone and change root "Hips" position...')

        leg_step = 0.01
        while self.step(self.timeStep) != -1:
            if leg_orientation[3] > 1.5 or leg_orientation[3] < 0.1:
                leg_step = -leg_step
                if leg_orientation[3] < 0.1:
                    # mode root 'Hips' bone
                    if hips_position[0] < 0.0:
                        hips_position[0] = 0.5
                    else:
                        hips_position[0] = -0.5
                    self.skin.setBonePosition(0, hips_position, True)
            leg_orientation[3] += leg_step
            self.skin.setBoneOrientation(leg_bone_index, leg_orientation, False)


controller = Controller()
controller.run()
