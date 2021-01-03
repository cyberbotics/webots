# Copyright 1996-2021 Cyberbotics Ltd.
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
    zOffset = 0
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
        previousXGridIndex, previousZGridIndex = \
            WebotsObject.elevation.get_grid_indexes(OSMCoord.coordDictionnary[refs[offset - 1]].x,
                                                    OSMCoord.coordDictionnary[refs[offset - 1]].z)
        newRefs = refs
        if offset >= len(refs):
            return newRefs
        for i in range(max(offset, 1), len(refs)):
            if not refs[i] in OSMCoord.coordDictionnary or not refs[i - 1] in OSMCoord.coordDictionnary:
                continue
            currentXGridIndex, currentZGridIndex = WebotsObject.elevation.get_grid_indexes(OSMCoord.coordDictionnary[refs[i]].x,
                                                                                           OSMCoord.coordDictionnary[refs[i]].z)
            if not previousXGridIndex == currentXGridIndex and currentXGridIndex > 0 and previousXGridIndex > 0:
                d1x = math.fabs(WebotsObject.elevation.elevationArray[min(previousXGridIndex, currentXGridIndex)]['x'] -
                                OSMCoord.coordDictionnary[refs[i]].x)
                d2x = math.fabs(WebotsObject.elevation.elevationArray[min(previousXGridIndex, currentXGridIndex)]['x'] -
                                OSMCoord.coordDictionnary[refs[i - 1]].x)
                if d1x > 0.01 and d2x > 0.01:
                    # need to add an intermediate point if none of the points are already very close from the border
                    newCoordX = (d2x / (d1x + d2x)) * OSMCoord.coordDictionnary[refs[i]].x + \
                        (d1x / (d1x + d2x)) * OSMCoord.coordDictionnary[refs[i - 1]].x
                    newCoordZ = (d2x / (d1x + d2x)) * OSMCoord.coordDictionnary[refs[i]].z + \
                        (d1x / (d1x + d2x)) * OSMCoord.coordDictionnary[refs[i - 1]].z
                    newOSMID = OSMCoord.add_new_coord_to_list(newCoordX, newCoordZ)
                    newRefs.insert(i, newOSMID)
                    return cls.add_intermediate_point_where_needed(newRefs, i)
            if not previousZGridIndex == currentZGridIndex and currentZGridIndex > 0 and previousZGridIndex > 0:
                d1z = math.fabs(WebotsObject.elevation.elevationArray[max(previousZGridIndex, currentZGridIndex)]['z'] -
                                OSMCoord.coordDictionnary[refs[i]].z)
                d2z = math.fabs(WebotsObject.elevation.elevationArray[max(previousZGridIndex, currentZGridIndex)]['z'] -
                                OSMCoord.coordDictionnary[refs[i - 1]].z)
                if d1z > 0.01 and d2z > 0.01:
                    # need to add an intermediate point if none of the points are already very close from the border
                    newCoordX = (d2z / (d1z + d2z)) * OSMCoord.coordDictionnary[refs[i]].x +\
                        (d1z / (d1z + d2z)) * OSMCoord.coordDictionnary[refs[i - 1]].x
                    newCoordZ = (d2z / (d1z + d2z)) * OSMCoord.coordDictionnary[refs[i]].z +\
                        (d1z / (d1z + d2z)) * OSMCoord.coordDictionnary[refs[i - 1]].z
                    newOSMID = OSMCoord.add_new_coord_to_list(newCoordX, newCoordZ)
                    newRefs.insert(i, newOSMID)
                    return cls.add_intermediate_point_where_needed(newRefs, i)
            newCoordX, newCoordZ = get_intersection(OSMCoord.coordDictionnary[refs[i]].x,
                                                    OSMCoord.coordDictionnary[refs[i]].z,
                                                    OSMCoord.coordDictionnary[refs[i - 1]].x,
                                                    OSMCoord.coordDictionnary[refs[i - 1]].z,
                                                    WebotsObject.elevation.elevationArray[currentXGridIndex]['x'],
                                                    WebotsObject.elevation.elevationArray[currentZGridIndex]['z'],
                                                    WebotsObject.elevation.elevationArray[currentXGridIndex]['x'] +
                                                    WebotsObject.elevation.xStep,
                                                    WebotsObject.elevation.elevationArray[currentZGridIndex]['z'] +
                                                    WebotsObject.elevation.zStep)
            if newCoordX is not None and newCoordZ is not None:  # add point if intersect the triangle
                newOSMID = OSMCoord.add_new_coord_to_list(newCoordX, newCoordZ)
                newRefs.insert(i, newOSMID)
                return cls.add_intermediate_point_where_needed(newRefs, i + 2)
            previousXGridIndex = currentXGridIndex
            previousZGridIndex = currentZGridIndex
        return newRefs
