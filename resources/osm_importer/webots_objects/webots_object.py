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

"""Contain the WebotsObject class."""

from osm_objects import OSMCoord
from utils.misc_utils import get_intersection

import math
import sys


class WebotsObject(object):
    """Abstract Webots object class."""

    layerHeight = 0.0  # height of 1 layer
    enable3D = False
    removalRadius = 0.0

    xOffset = 0
    yOffset = 0
    elevation = None

    @classmethod
    def export(cls, file):
        """Export all the objects from the objects list."""
        sys.stderr.write("Warning: function 'export' not implemented in class " + cls.__name__ + "\n")

    @classmethod
    def add_intermediate_point_where_needed(cls, refs, offset=1):
        """Add intermediate point when crossing the grid of the ElevationGrid."""
        if len(refs) < offset or not refs[offset - 1] in OSMCoord.coordDictionnary or WebotsObject.elevation is None:
            return refs
        previousXGridIndex, previousYGridIndex = \
            WebotsObject.elevation.get_grid_indexes(OSMCoord.coordDictionnary[refs[offset - 1]].x,
                                                    OSMCoord.coordDictionnary[refs[offset - 1]].y)
        newRefs = refs
        if offset >= len(refs):
            return newRefs
        for i in range(max(offset, 1), len(refs)):
            if not refs[i] in OSMCoord.coordDictionnary or not refs[i - 1] in OSMCoord.coordDictionnary:
                continue
            currentXGridIndex, currentYGridIndex = WebotsObject.elevation.get_grid_indexes(OSMCoord.coordDictionnary[refs[i]].x,
                                                                                           OSMCoord.coordDictionnary[refs[i]].y)
            if not previousXGridIndex == currentXGridIndex and currentXGridIndex > 0 and previousXGridIndex > 0:
                d1x = math.fabs(WebotsObject.elevation.elevationArray[min(previousXGridIndex, currentXGridIndex)]['x'] -
                                OSMCoord.coordDictionnary[refs[i]].x)
                d2x = math.fabs(WebotsObject.elevation.elevationArray[min(previousXGridIndex, currentXGridIndex)]['x'] -
                                OSMCoord.coordDictionnary[refs[i - 1]].x)
                if d1x > 0.01 and d2x > 0.01:
                    # need to add an intermediate point if none of the points are already very close from the border
                    newCoordX = (d2x / (d1x + d2x)) * OSMCoord.coordDictionnary[refs[i]].x + \
                        (d1x / (d1x + d2x)) * OSMCoord.coordDictionnary[refs[i - 1]].x
                    newCoordY = (d2x / (d1x + d2x)) * OSMCoord.coordDictionnary[refs[i]].y + \
                        (d1x / (d1x + d2x)) * OSMCoord.coordDictionnary[refs[i - 1]].y
                    newOSMID = OSMCoord.add_new_coord_to_list(newCoordX, newCoordY)
                    newRefs.insert(i, newOSMID)
                    return cls.add_intermediate_point_where_needed(newRefs, i)
            if not previousYGridIndex == currentYGridIndex and currentYGridIndex > 0 and previousYGridIndex > 0:
                d1y = math.fabs(WebotsObject.elevation.elevationArray[max(previousYGridIndex, currentYGridIndex)]['y'] -
                                OSMCoord.coordDictionnary[refs[i]].y)
                d2y = math.fabs(WebotsObject.elevation.elevationArray[max(previousYGridIndex, currentYGridIndex)]['y'] -
                                OSMCoord.coordDictionnary[refs[i - 1]].y)
                if d1y > 0.01 and d2y > 0.01:
                    # need to add an intermediate point if none of the points are already very close from the border
                    newCoordX = (d2y / (d1y + d2y)) * OSMCoord.coordDictionnary[refs[i]].x +\
                        (d1y / (d1y + d2y)) * OSMCoord.coordDictionnary[refs[i - 1]].x
                    newCoordy = (d2y / (d1y + d2y)) * OSMCoord.coordDictionnary[refs[i]].y +\
                        (d1y / (d1y + d2y)) * OSMCoord.coordDictionnary[refs[i - 1]].y
                    newOSMID = OSMCoord.add_new_coord_to_list(newCoordX, newCoordY)
                    newRefs.insert(i, newOSMID)
                    return cls.add_intermediate_point_where_needed(newRefs, i)
            newCoordX, newCoordY = get_intersection(OSMCoord.coordDictionnary[refs[i]].x,
                                                    OSMCoord.coordDictionnary[refs[i]].y,
                                                    OSMCoord.coordDictionnary[refs[i - 1]].x,
                                                    OSMCoord.coordDictionnary[refs[i - 1]].y,
                                                    WebotsObject.elevation.elevationArray[currentXGridIndex]['x'],
                                                    WebotsObject.elevation.elevationArray[currentYGridIndex]['y'],
                                                    WebotsObject.elevation.elevationArray[currentXGridIndex]['x'] +
                                                    WebotsObject.elevation.xStep,
                                                    WebotsObject.elevation.elevationArray[currentYGridIndex]['y'] +
                                                    WebotsObject.elevation.yStep)
            if newCoordX is not None and newCoordY is not None:  # add point if intersect the triangle
                newOSMID = OSMCoord.add_new_coord_to_list(newCoordX, newCoordy)
                newRefs.insert(i, newOSMID)
                return cls.add_intermediate_point_where_needed(newRefs, i + 2)
            previousXGridIndex = currentXGridIndex
            previousYGridIndex = currentYGridIndex
        return newRefs
