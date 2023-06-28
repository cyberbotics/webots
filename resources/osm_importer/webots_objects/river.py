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

"""Contains the River class."""

from webots_objects.webots_object import WebotsObject
from webots_objects.road import Road

from settings import Settings
from osm_objects import OSMCoord
from utils.misc_utils import extract_float_from_string, clean_string, protect_def_name


class River(WebotsObject):
    """River class representing a river."""

    list = []

    def __init__(self):
        """Initialize the river."""
        self.OSMID = 0
        self.type = ""
        self.ref = 0
        self.width = 2
        self.name = ""

    @staticmethod
    def add_to_list(osmid, tags, ref):
        """Add a new river to the list of rivers."""
        if 'waterway' in tags:
            settingsSection = Settings.get_section('waterway', tags['waterway'])
            if settingsSection is None:
                return
            river = River()
            river.OSMID = osmid
            river.type = tags['waterway']
            river.ref = ref
            if 'name' in tags:
                river.name = clean_string(tags['name'])
            if 'width' in tags:
                river.width = extract_float_from_string(tags['width'])
            elif Settings.has_option(settingsSection, 'width'):
                river.width = Settings.getfloat(settingsSection, 'width')
            if WebotsObject.enable3D:
                river.ref = River.add_intermediate_point_where_needed(river.ref)
            River.list.append(river)

    @classmethod
    def export(cls, file):
        """Export all the rivers from the rivers list."""
        for river in River.list[:]:
            if not river.ref:
                River.list.remove(river)
                continue

            if WebotsObject.removalRadius > 0.0:
                # Check that the river is inside the scope of a road,
                # otherwise, remove it from the river list.
                if not Road.are_coords_close_to_some_road_waypoint(river.ref):
                    River.list.remove(river)
                    continue

            if not river.name == "":
                file.write("DEF " + protect_def_name(river.name) + " " + "Transform {\n")
            else:
                file.write("Transform {\n")
            file.write("  translation %.2lf %.2lf %.2lf\n" % (OSMCoord.coordDictionnary[river.ref[0]].x,
                                                              OSMCoord.coordDictionnary[river.ref[0]].y,
                                                              OSMCoord.coordDictionnary[river.ref[0]].z))
            file.write("  children [\n")
            file.write("    Shape {\n")
            file.write("      appearance PBRAppearance {\n")
            file.write("        baseColor 0.3 0.5 0.8\n")
            file.write("        roughness 0.3\n")
            file.write("        metalness 0\n")
            file.write("      }\n")
            file.write("      geometry Extrusion {\n")
            file.write("        crossSection [\n")
            file.write("          %.2f 0\n" % (-river.width / 2))
            file.write("          %.2f 0.5\n" % (-river.width / 2))
            file.write("          %.2f 0.5\n" % (river.width / 2))
            file.write("          %.2f 0\n" % (river.width / 2))
            file.write("          %.2f 0\n" % (-river.width / 2))
            file.write("        ]\n")
            file.write("        spine [\n")
            for ref in river.ref:
                if ref in OSMCoord.coordDictionnary:
                    file.write("          %.2f %.2f %.2f,\n" % (OSMCoord.coordDictionnary[ref].x -
                                                                OSMCoord.coordDictionnary[river.ref[0]].x,
                                                                OSMCoord.coordDictionnary[ref].y -
                                                                OSMCoord.coordDictionnary[river.ref[0]].y,
                                                                OSMCoord.coordDictionnary[ref].z -
                                                                OSMCoord.coordDictionnary[river.ref[0]].z))
                else:
                    print("Warning: node " + str(ref) + " not referenced.")
            file.write("  ]\n")

            file.write("          splineSubdivision 0\n")
            file.write("      }\n")
            file.write("      castShadows FALSE\n")
            file.write("    }\n")
            file.write("  ]\n")
            file.write("}\n")
