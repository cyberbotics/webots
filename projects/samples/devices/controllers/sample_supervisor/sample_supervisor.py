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
An example of a supervisor controller.
"""

from controller import Supervisor
import math


class Controller(Supervisor):
    SPEED = 6
    timeStep = 64

    def __init__(self):
        super(Controller, self).__init__()

    def run(self):
        self.root_node = self.getRoot()
        root_children_field = self.root_node.getField('children')
        n = root_children_field.getCount()
        print(f'This world contains {n} nodes:')
        # check what type of nodes are present in the world
        for i in range(n):
            node = root_children_field.getMFNode(i)
            print(f'-> {node.getTypeName()}')
        print()

        # get the content of the 'gravity' field of the 'WorldInfo' node
        node = root_children_field.getMFNode(0)
        field = node.getField('gravity')
        gravity = field.getSFFloat()
        print(f'WorldInfo.gravity = {gravity}\n')

        # use a label to display information in the 3D view
        self.setLabel(0, 'Going to move the location of the PointLight\nin 2 seconds (simulation time)...',
                      0.0, 0.0, 0.1, 0x00ff00, 0.1, 'Georgia')
        # move the 'PointLight' node after waiting 2 seconds
        print('Going to move the location of the PointLight in 2 seconds (simulation time)...')
        self.step(2000)  # wait for 2 seconds
        node = root_children_field.getMFNode(3)  # PointLight
        field = node.getField('location')
        location = [0.5, 0.5, 0.3]
        field.setSFVec3f(location)

        # import a new sphere node after waiting 2 seconds
        self.setLabel(0, 'Going to import a Sphere in 2 seconds (simulation time)...',
                      0.0, 0.0, 0.1, 0x00FF00, 0.1, 'Georgia')
        print('Going to import a Sphere in 2 seconds (simulation time)...')
        self.step(2000)
        root_children_field.importMFNodeFromString(-1,  # import at the end of the root children field
                                                   'Pose { children [ Shape { appearance PBRAppearance { } '
                                                   'geometry Sphere { radius 0.1 subdivision 3 } } ] }')

        # main simulation loop
        self.setLabel(0, 'Going to move the Sphere in 2 seconds (simulation time)...',
                      0.0, 0.0, 0.1, 0x00FF00, 0.1, 'Georgia')
        print('Going to move the Sphere in 2 seconds (simulation time)...')
        self.step(2000)
        self.setLabel(0, '', 0.0, 0.0, 0.0, 0x00FF00, 0.0, 'Georgia')
        translation = [0.0, 0.0, 0.0]
        # get the last node of the root children field (the Sphere)
        node = root_children_field.getMFNode(-1)
        field = node.getField('translation')
        while self.step(32) != -1:
            # move the Sphere node in a circle of 0.3m of radius
            translation[0] = 0.3 * math.cos(self.getTime())
            translation[1] = 0.3 * math.sin(self.getTime())
            field.setSFVec3f(translation)


controller = Controller()
controller.run()
