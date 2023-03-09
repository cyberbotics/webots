#!/usr/bin/env python3
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

"""This module gather the OSM objects classes."""

import math

from projection import Projection
from utils.misc_utils import length2D


class OSMAbstractObject(object):
    """Represent an abstract OSM object."""

    def __init__(self):
        """Initialize the object."""
        self.OSMID = 0


class OSMCoord(OSMAbstractObject):
    """Represent an OSM coordinate."""

    coordDictionnary = {}
    newCoordOSMID = 0

    def __init__(self):
        """Initialize the coordinate."""
        super(OSMCoord, self).__init__()
        self.long = 0
        self.lat = 0
        self.x = 0
        self.y = 0
        self.z = 0
        self.tilt = 0

    @staticmethod
    def add(osmid, long, lat):
        """Add a new coordinate to the list from longitude latitude."""
        coord = OSMCoord()
        coord.OSMID = osmid
        coord.long = long
        coord.lat = lat
        coord.x, coord.y = Projection.project(coord.long, coord.lat)
        OSMCoord.coordDictionnary[osmid] = coord

    @staticmethod
    def addFromXY(osmid, x, y, z):
        """Add a new coordinate to the list from X Y."""
        coord = OSMCoord()
        coord.OSMID = osmid
        coord.x = x
        coord.z = z
        coord.y = y
        OSMCoord.coordDictionnary[osmid] = coord

    @staticmethod
    def add_new_coord_to_list(x, y, z=0):
        """Add a new coordinate to the list from X Y and create s new OSMID."""
        while OSMCoord.newCoordOSMID in OSMCoord.coordDictionnary:
            OSMCoord.newCoordOSMID = OSMCoord.newCoordOSMID + 1
        OSMCoord.addFromXY(OSMCoord.newCoordOSMID, x, y, z)
        return OSMCoord.newCoordOSMID

    @staticmethod
    def center_coordinates(minlat, minlon, maxlat, maxlon):
        """Center the coordinate around (0,0) and returns the offsets between earth and local world coordinate."""
        x1, y1 = Projection.project(minlon, minlat)
        x2, y2 = Projection.project(maxlon, maxlat)
        xOffset = (x1 + x2) / 2
        yOffset = (y1 + y2) / 2
        for osmid in OSMCoord.coordDictionnary:
            # inverse X, because OSM and Webots X are inversed
            OSMCoord.coordDictionnary[osmid].x = OSMCoord.coordDictionnary[osmid].x - xOffset
            OSMCoord.coordDictionnary[osmid].y = OSMCoord.coordDictionnary[osmid].y - yOffset
        return xOffset, yOffset

    @staticmethod
    def get_min_and_max_coord(refs):
        """Return the coordinates bounds."""
        if not refs:
            return (0, 0, 0, 0)
        xMin = OSMCoord.coordDictionnary[refs[0]].x
        xMax = OSMCoord.coordDictionnary[refs[0]].x
        yMin = OSMCoord.coordDictionnary[refs[0]].y
        yMax = OSMCoord.coordDictionnary[refs[0]].y
        for ref in refs:
            if xMin < OSMCoord.coordDictionnary[ref].x:
                xMin = OSMCoord.coordDictionnary[ref].x
            if xMax > OSMCoord.coordDictionnary[ref].x:
                xMax = OSMCoord.coordDictionnary[ref].x
            if yMin < OSMCoord.coordDictionnary[ref].y:
                yMin = OSMCoord.coordDictionnary[ref].y
            if yMax > OSMCoord.coordDictionnary[ref].y:
                yMax = OSMCoord.coordDictionnary[ref].y
        return (xMin, xMax, yMin, yMax)


class OSMNode(OSMCoord):
    """Represent an OSM node."""

    nodeDictionnary = {}

    def __init__(self):
        """Initialize the node."""
        super(OSMNode, self).__init__()
        self.tags = ""

    @staticmethod
    def add(osmid, long, lat, tags):
        """Add a new node to the list from longitude latitude."""
        node = OSMNode()
        node.OSMID = osmid
        node.long = long
        node.lat = lat
        node.x, node.y = Projection.project(node.long, node.lat)
        node.tags = tags
        OSMNode.nodeDictionnary[osmid] = node


class OSMMultipolygon(object):
    """Represent an OSM multipolygon."""

    multipolygonList = []
    disableMultipolygonBuildings = False

    def __init__(self):
        """Initialize the multipolygon."""
        self.OSMID = 0
        self.tags = ""
        self.ref = []

    @staticmethod
    def add(osmid, tags, members, wayRefList):
        """Add a new Multipolygon to the list."""
        multipolygon = OSMMultipolygon()
        multipolygon.OSMID = osmid
        multipolygon.tags = tags

        for member in members:
            reference = member['reference']
            type = member['type']
            role = member['role']
            if type == 'way' and role == 'outer' and reference in wayRefList:
                refToAdd = wayRefList[reference]
                # if the new ref to be added are not defined in the same direction than the one already added
                if not len(multipolygon.ref) == 0 and not multipolygon.ref[-1] == refToAdd[0]:
                    refToAdd.reverse()
                    if not multipolygon.ref[-1] == refToAdd[0]:
                        multipolygon.ref.reverse()
                        if not multipolygon.ref[-1] == refToAdd[0]:
                            refToAdd.reverse()
                if len(multipolygon.ref) == 0:
                    multipolygon.ref = multipolygon.ref + refToAdd
                else:
                    for ref in refToAdd:
                        # if this point is already in the list (but it is equal to previous one) => loop closed
                        if ref in multipolygon.ref and not ref == multipolygon.ref[-1]:
                            multipolygon.ref = multipolygon.ref + refToAdd
                            # clean multipolygon
                            for index in range(len(multipolygon.ref) - 1, 0, -1):
                                if multipolygon.ref[index] == multipolygon.ref[index - 1]:
                                    del multipolygon.ref[index]
                            OSMMultipolygon.multipolygonList.append(multipolygon)  # add this multipolygon to the list
                            break
                    # if loop not close add this part of multipolygon and continue
                    if len(OSMMultipolygon.multipolygonList) > 0 and not OSMMultipolygon.multipolygonList[-1] == multipolygon:
                        multipolygon.ref = multipolygon.ref + refToAdd
                    # otherwise this multipolygon is done
                    else:
                        break
        # this multipolygon is only partial (but we still want to add it to the list)
        if (not len(multipolygon.ref) == 0 and not
                (len(OSMMultipolygon.multipolygonList) > 0 and OSMMultipolygon.multipolygonList[-1] == multipolygon)):
            OSMMultipolygon.multipolygonList.append(multipolygon)

    @staticmethod
    def sum_distances_to_coords(coordlist, x, y, threshold):
        """Return the sum of the distances to each point closer than the threshold."""
        total = 0
        for index in coordlist:
            distance = length2D(coordlist[index].x - x, coordlist[index].y - y)
            if distance <= threshold:
                total = total + distance
        return total

    def add_intermediate_point(self):
        """If last and first points are not the same we need to compute an intermediate point location."""
        """The point is used to close the polygon."""
        coordBegin = OSMCoord.coordDictionnary[self.ref[0]]
        coordEnd = OSMCoord.coordDictionnary[self.ref[-1]]
        distance = length2D(coordBegin.x - coordEnd.x, coordBegin.y - coordEnd.y)
        angle = math.atan2(coordBegin.y - coordEnd.y, coordBegin.x - coordEnd.x)

        # there is two possible 'optimal' intermediate points
        # we select the one that is the farthest from all the other coord (=>inside the lake)
        x1 = math.cos(math.pi / 2 + angle) * (distance / 2) + (coordBegin.x + coordEnd.x) / 2
        y1 = math.sin(math.pi / 2 + angle) * (distance / 2) + (coordBegin.y + coordEnd.y) / 2
        x2 = -math.cos(math.pi / 2 + angle) * (distance / 2) + (coordBegin.x + coordEnd.x) / 2
        y2 = -math.sin(math.pi / 2 + angle) * (distance / 2) + (coordBegin.y + coordEnd.y) / 2
        distanceSum1 = OSMMultipolygon.sum_distances_to_coords(OSMCoord.coordDictionnary, x1, y1, 2000)
        distanceSum2 = OSMMultipolygon.sum_distances_to_coords(OSMCoord.coordDictionnary, x2, y2, 2000)
        if distanceSum1 < distanceSum2:
            x = x1
            y = y1
        else:
            x = x2
            y = y2
        self.ref.append(OSMCoord.add_new_coord_to_list(x, y))

    @staticmethod
    def process(disableMultipolygonBuildings):
        """Process all the multipolygon (mainly assure that they are closed)."""
        OSMMultipolygon.disableMultipolygonBuildings = disableMultipolygonBuildings
        for multipolygon in OSMMultipolygon.multipolygonList:
            if not multipolygon.ref[0] == multipolygon.ref[-1]:
                multipolygon.add_intermediate_point()
