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

"""Contains the Tree class."""

from webots_objects.webots_object import WebotsObject
from webots_objects.road import Road

import math
import random


class Tree(WebotsObject):
    """Tree class representing an isolated tree."""

    list = []
    nameIndex = 1
    needleLeavesTypes = ["spruce", "white pine"]
    broadLeavesTypes = ["birch tree", "cherry tree", "crab apple tree", "hackberry tree", "hazel tree", "oak tree"]

    def __init__(self):
        """Export all the trees from the trees list."""
        self.tags = ""
        self.coord = None
        self.height = None
        self.radius = None
        self.leafType = None

    @classmethod
    def export(cls, file):
        """Export all the trees from the trees list."""
        for tree in Tree.list[:]:

            if WebotsObject.removalRadius > 0.0:
                # Check that the tree is inside the scope of a road,
                # otherwise, remove it from the tree list.
                if not Road.are_coords_close_to_some_road_waypoint([[tree.coord.x, tree.coord.y]], areOSMReferences=False):
                    Tree.list.remove(tree)
                    continue

            file.write('SimpleTree {\n')
            if tree.leafType == 'needleleaved':
                file.write(' type "%s"\n' % random.choice(Tree.needleLeavesTypes))
            elif tree.leafType == 'broadleaved':
                file.write(' type "%s"\n' % random.choice(Tree.broadLeavesTypes))
            else:
                file.write('  type "random"\n')
            file.write('  rotation 0 0 1 %.3f\n' % (random.random() * 2 * math.pi))
            file.write('  translation %.2f %.2f %.2f\n' % (tree.coord.x, tree.coord.y, tree.coord.z))
            file.write('  name "tree(%d)"\n' % Tree.nameIndex)
            Tree.nameIndex += 1
            file.write('  enableBoundingObject FALSE\n')
            if tree.height is not None:
                file.write('  height %.3f\n' % (tree.height))
            if tree.radius is not None:
                file.write('  radius %.3f\n' % (tree.radius))
            elif tree.height is not None:
                file.write('  radius %.3f\n' % (tree.height / 2.0))
            file.write('}\n')
