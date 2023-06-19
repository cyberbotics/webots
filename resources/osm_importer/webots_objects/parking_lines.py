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

"""Contains the ParkingLines class."""

from webots_objects.webots_object import WebotsObject
from webots_objects.road import Road

from osm_objects import OSMCoord
from utils.vector import Vector2D

import math


class ParkingLines(WebotsObject):
    """ParkingLines class representing a parking lines."""

    list = []

    def __init__(self):
        """Initialize the parking lines."""
        self.OSMID = 0
        self.ref = 0

    @staticmethod
    def add_to_list(osmid, tags, ref):
        """Add a new parking lines to the list of parking lines."""
        parkingLines = ParkingLines()
        parkingLines.OSMID = osmid
        parkingLines.ref = ref
        ParkingLines.list.append(parkingLines)

    @classmethod
    def export(cls, file):
        """Export all the parking lines from the parking lines list."""
        for parkingLines in ParkingLines.list[:]:
            if len(parkingLines.ref) != 2:
                ParkingLines.list.remove(parkingLines)
                continue

            if WebotsObject.removalRadius > 0.0:
                # Check that the parking lines is inside the scope of a road,
                # otherwise, remove it from the parking lines list.
                if not Road.are_coords_close_to_some_road_waypoint(parkingLines.ref):
                    ParkingLines.list.remove(parkingLines)
                    continue

            # Coordinates extration
            c0 = (OSMCoord.coordDictionnary[parkingLines.ref[0]].x,
                  OSMCoord.coordDictionnary[parkingLines.ref[0]].y,
                  OSMCoord.coordDictionnary[parkingLines.ref[0]].z)
            c1 = (OSMCoord.coordDictionnary[parkingLines.ref[1]].x,
                  OSMCoord.coordDictionnary[parkingLines.ref[1]].y,
                  OSMCoord.coordDictionnary[parkingLines.ref[1]].z)

            # Compute the length and the angle
            v0 = Vector2D(c0[0], c0[2])
            v1 = Vector2D(c1[0], c1[2])
            deltaV = v0 - v1
            length = deltaV.norm()
            angle = - deltaV.angle() + math.pi
            expectedCarParkWidth = 2.4
            # cf. http://stackoverflow.com/questions/3950372/round-with-integer-division
            nCarParks = int((length + expectedCarParkWidth // 2) // expectedCarParkWidth)
            carParkWidth = length / nCarParks

            file.write("ParkingLines {\n")
            file.write("  translation %.2lf %.2lf %.2lf\n" % (c0[0], c0[1] + 0.01, c0[2]))
            file.write("  rotation 0 1 0 %.2lf\n" % (angle))
            file.write("  numberOfCarParks %d\n" % (nCarParks))
            file.write("  carParkWidth %f\n" % (carParkWidth))
            file.write("}\n")
