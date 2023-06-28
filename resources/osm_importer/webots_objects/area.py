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

"""Contains the Area class."""

from webots_objects.webots_object import WebotsObject
from webots_objects.road import Road
from webots_objects.tree import Tree

from settings import Settings
from osm_objects import OSMCoord

from utils.misc_utils import clean_string, protect_def_name

import os
import random


class Area(WebotsObject):
    """Area class representing for example a forest area or a water area."""

    list = []
    noForests = False

    def __init__(self):
        """Initialize the area."""
        self.OSMID = 0
        self.type = ""
        self.ref = []
        self.color = [0, 0, 0]
        self.transparency = 0
        self.texture = ""
        self.name = ""
        self.density = 0.1
        self.leafType = None

    @staticmethod
    def add_to_list(osmid, type, tags, ref):
        """Add a new area to the list of areas."""
        settingsSection = Settings.get_section('area', type)
        if settingsSection is None:
            return
        area = Area()
        area.OSMID = osmid
        area.ref = ref
        area.type = type
        if 'name' in tags:
            area.name = clean_string(tags['name'])
        if 'leaf_type' in tags:
            area.leafType = tags['leaf_type']
        if Settings.has_option(settingsSection, 'texture'):
            area.texture = Settings.get(settingsSection, 'texture')
        if Settings.has_option(settingsSection, 'transparency'):
            area.transparency = Settings.getfloat(settingsSection, 'transparency')
        if Settings.has_option(settingsSection, 'redComponent'):
            area.color[0] = Settings.getfloat(settingsSection, 'redComponent')
        if Settings.has_option(settingsSection, 'greenComponent'):
            area.color[1] = Settings.getfloat(settingsSection, 'greenComponent')
        if Settings.has_option(settingsSection, 'blueComponent'):
            area.color[2] = Settings.getfloat(settingsSection, 'blueComponent')
        if Settings.has_option(settingsSection, 'density'):
            area.density = Settings.getfloat(settingsSection, 'density')
        Area.list.append(area)

    @staticmethod
    def are_references_clockwise(referenceList):
        """Check if a list of points is defined in a clockwise maner."""
        total = 0
        for i in range(0, len(referenceList)):
            total = total + (OSMCoord.coordDictionnary[referenceList[i]].x -
                             OSMCoord.coordDictionnary[referenceList[i - 1]].x) * \
                (OSMCoord.coordDictionnary[referenceList[i]].y +
                 OSMCoord.coordDictionnary[referenceList[i - 1]].y)
        if total >= 0:
            return True
        else:
            return False

    @staticmethod
    def is_point_in_polygon(x, y, polygon):
        """Determine if a point is inside a given polygon or not."""
        """Polygon is a list of (x,y) pairs."""
        n = len(polygon)
        inside = False
        p1x, p1y = polygon[0]
        for i in range(n + 1):
            p2x, p2y = polygon[i % n]
            if y > min(p1y, p2y):
                if y <= max(p1y, p2y):
                    if x <= max(p1x, p2x):
                        if p1y != p2y:
                            xinters = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                        if p1x == p2x or x <= xinters:
                            inside = not inside
            p1x, p1y = p2x, p2y
        return inside

    @staticmethod
    def draw_area(file, refs, red=1, green=0, blue=0, defName="", transparency=0.0, texture="", drawFlat=False,
                  verticalOffset=0.0):
        """Draw an area."""
        if len(refs) < 3:
            return
        if defName:
            file.write("DEF " + defName + " " + "Transform {\n")
        else:
            file.write("Transform {\n")
        file.write("  translation %f %f 0\n" %
                   (OSMCoord.coordDictionnary[refs[0]].x, OSMCoord.coordDictionnary[refs[0]].y))
        file.write("  children [\n")
        file.write("    Shape {\n")
        file.write("      appearance PBRAppearance {\n")
        if transparency > 0:
            file.write("        transparency " + str(transparency) + "\n")
        file.write("        roughness 1\n")
        file.write("        metalness 0\n")
        if texture:
            file.write("        baseColorMap ImageTexture {\n")
            file.write("          url [\n")
            file.write("            \"" + texture + "\"\n")
            file.write("          ]\n")
            file.write("        }\n")
            xMin, xMax, yMin, yMax = OSMCoord.get_min_and_max_coord(refs)
            scale = max(abs(round(xMax - xMin)), abs(round(yMax - yMin)))
            file.write("        textureTransform TextureTransform {\n")
            file.write("          scale %.2f %.2f\n" % (scale, scale))
            file.write("        }\n")
        else:
            file.write("        baseColor " + str(red) + " " + str(green) + " " + str(blue) + "\n")
        file.write("      }\n")
        file.write("      geometry IndexedFaceSet {\n")
        file.write("        coord Coordinate {\n")
        file.write("          point [\n")
        if drawFlat:
            height = 0
            for ref in refs:
                if ref in OSMCoord.coordDictionnary:
                    height = height + OSMCoord.coordDictionnary[ref].z
            height = height / len(refs)
        for ref in refs:
            if ref in OSMCoord.coordDictionnary:
                if drawFlat:
                    file.write("            %.2f %.2f %.2f,\n" % (OSMCoord.coordDictionnary[ref].x -
                                                                  OSMCoord.coordDictionnary[refs[0]].x,
                                                                  OSMCoord.coordDictionnary[ref].y -
                                                                  OSMCoord.coordDictionnary[refs[0]].y,
                                                                  height + verticalOffset))
                else:
                    file.write("            %.2f %.2f %.2f,\n" % (OSMCoord.coordDictionnary[ref].x -
                                                                  OSMCoord.coordDictionnary[refs[0]].x,
                                                                  OSMCoord.coordDictionnary[ref].y -
                                                                  OSMCoord.coordDictionnary[refs[0]].y,
                                                                  OSMCoord.coordDictionnary[ref].z + verticalOffset,))
            else:
                print("Warning: node " + str(ref) + " not referenced.")
        file.write("          ]\n")
        file.write("        }\n")
        if Area.are_references_clockwise(refs) is True:
            file.write("        ccw FALSE\n")
        file.write("        coordIndex [\n")
        for i in range(0, len(refs)):
            file.write("          " + str(i) + "\n")
        file.write("          -1\n")
        file.write("        ]\n")
        file.write("      }\n")
        file.write("      castShadows FALSE\n")
        file.write("    }\n")
        file.write("  ]\n")
        file.write("}\n")

    @classmethod
    def export(cls, file, noParkings):
        """Export all the areas from the areas list."""
        for area in Area.list[:]:
            if noParkings and area.type == 'parking':
                Area.list.remove(area)
                continue

            if WebotsObject.removalRadius > 0.0:
                # Check that the area is inside the scope of a road,
                # otherwise, remove it from the area list.
                if not Road.are_coords_close_to_some_road_waypoint(area.ref):
                    Area.list.remove(area)
                    continue

            defName = ""
            if not area.name == "":
                defName = area.name
            else:
                defName = area.type
            defName = protect_def_name(defName)
            if area.type == 'forest' and not Area.noForests:
                treesFile = area.generate_tree_file(os.path.dirname(os.path.abspath(file.name)))
                if file is not None:
                    file.write("Forest {\n")
                    file.write("  shape [\n")
                    if Area.are_references_clockwise(area.ref) is True:
                        for ref in area.ref:
                            file.write("    %.2f %.2f, " %
                                       (OSMCoord.coordDictionnary[ref].x, -OSMCoord.coordDictionnary[ref].y))
                    else:
                        for ref in reversed(area.ref):
                            file.write("    %.2f %.2f, " %
                                       (OSMCoord.coordDictionnary[ref].x, -OSMCoord.coordDictionnary[ref].y))
                    file.write("\n]\n")
                    if area.leafType == "needleleaved":
                        file.write(' type "%s"\n' % random.choice(Tree.needleLeavesTypes))
                    elif area.leafType == "broadleaved":
                        file.write(' type "%s"\n' % random.choice(Tree.broadLeavesTypes))
                    else:
                        file.write("  type \"random\"\n")
                    file.write("  treesFiles [\n")
                    file.write("    " + "\"" + treesFile + "\"" + "\n")
                    file.write("  ]\n")
                    file.write("}\n")
            else:
                verticalOffset = -0.01 if area.type == 'parking' else 0.0
                drawFlat = True if area.type == 'water' else False
                Area.draw_area(file, area.ref, area.color[0], area.color[1], area.color[2], defName, area.transparency,
                               area.texture, verticalOffset=verticalOffset, drawFlat=drawFlat)

    def generate_tree_file(self, folder):
        """Generate the 'forest' file which contains the tree positions and is used by the 'Forest' PROTO."""
        treeNumber = 0
        polygon = []
        xMin, xMax, yMin, yMax = OSMCoord.get_min_and_max_coord(self.ref)
        for ref in self.ref:
            polygon.append([OSMCoord.coordDictionnary[ref].x, OSMCoord.coordDictionnary[ref].y])

        numberOfTree = int(round((xMax - xMin) * (yMax - yMin)) * self.density)
        if not os.path.exists(folder + '/forest'):
            os.makedirs(folder + '/forest')
        forestRelativePath = 'forest/' + str(self.OSMID) + '.forest'
        forestPath = folder + '/' + forestRelativePath
        with open(forestPath, 'w') as file:
            for index in range(0, numberOfTree):
                x = random.uniform(xMin, xMax)
                y = random.uniform(yMin, yMax)
                z = 0
                if WebotsObject.elevation is not None:
                    z = WebotsObject.elevation.interpolate_height(
                        x + WebotsObject.xOffset, y + WebotsObject.yOffset)
                if Area.is_point_in_polygon(x, y, polygon) is True:
                    treeNumber = treeNumber + 1
                    file.write("%.2f,%.2f,%.2f\n" % (x, -y, z))

        if treeNumber > 0:
            return forestRelativePath
        else:
            if os.path.isfile(forestPath):
                os.remove(forestPath)
            return None
