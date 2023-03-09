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

"""Importer for writing a Webots worlds from an Open Street Map file."""

import codecs
import optparse
import os
import re
import random
import sys

from elevation import Elevation
from osm_objects import OSMCoord
from parser_objects import Parser
from projection import Projection
from settings import Settings
from utils.misc_utils import print_header
from webots_objects.webots_object import WebotsObject
from webots_objects.area import Area
from webots_objects.barrier import Barrier
from webots_objects.building import Building
from webots_objects.parking_lines import ParkingLines
from webots_objects.river import River
from webots_objects.road import Road
from webots_objects.tree import Tree


def add_height_to_coordinates(elevation):
    """Compute the Y of each coordinate."""
    for osmid in OSMCoord.coordDictionnary:
        OSMCoord.coordDictionnary[osmid].z = elevation.interpolate_height(OSMCoord.coordDictionnary[osmid].x,
                                                                          OSMCoord.coordDictionnary[osmid].y)


# Parse the options.
optParser = optparse.OptionParser(usage="usage: %prog --input=file.osm [options]")
optParser.add_option("--input", dest="inFile", default="map.osm", help="specifies the osm file to open")
optParser.add_option("--output", dest="outFile", default="map.wbt", help="specifies the name of the generated world")
optParser.add_option("--config-file", dest="configFile", default="", help="specifies the config file to use")
optParser.add_option("--layer-height", type="float", dest="layer", default=5.0,
                     help="specifies the height of a layer (the 'layer' tag is ignored if set to 0)")
optParser.add_option("--no-forests", dest="noForests", action="store_true", default=False, help="does not generate forests")
optParser.add_option("--no-roads", dest="noRoads", action="store_true", default=False,
                     help="does not generate roads")
optParser.add_option("--no-areas", dest="noAreas", action="store_true", default=False,
                     help="does not generate areas (water, landuse, etc.)")
optParser.add_option("--no-parkings", dest="noParkings", action="store_true", default=False, help="does not generate parkings")
optParser.add_option("--no-trees", dest="noTrees", action="store_true", default=False, help="does not generate trees")
optParser.add_option("--no-barriers", dest="noBarriers", action="store_true", default=False,
                     help="does not generate barriers (fence, wall, etc.)")
optParser.add_option("--no-rivers", dest="noRivers", action="store_true", default=False, help="does not generate rivers")
optParser.add_option("--no-buildings", dest="noBuildings", action="store_true", default=False,
                     help="does not generate buildings")
optParser.add_option("--no-intersection-road-lines", dest="noIntersectionRoadLines", action="store_true", default=False,
                     help="does not generate road start and end lines at intersections")
optParser.add_option("--enable-3D", dest="enable3D", action="store_true", default=False, help="enables the third dimension")
optParser.add_option("--google-api-key", type="string", dest="googleAPIKey", default="",
                     help="specifies your key to access the Google Elevation API (required when 3D is enabled)")
optParser.add_option("--disable-multipolygon-buildings", dest="disableMultipolygonBuildings", action="store_true",
                     default=False, help="does not generate buildings from multipolygon")
optParser.add_option("--projection", type="string", dest="projection", default="",
                     help="specifies the projection parameters (an utm projection is used by default)")
optParser.add_option("--extract-projection", dest="extractProjection", action="store_true", default=False,
                     help="extracts the projection from the OSM file, displays it and exits.")
optParser.add_option("--removal-radius", dest="removalRadius", type="float", default=0.0,
                     help="specifies the radius around each road waypoint beyond which any object is removed.")
options, args = optParser.parse_args()

# Deal with required argument (1)
if not os.path.isfile(options.inFile):
    optParser.error("Invalid OSM input file.")

# determinist random seed
random.seed(0)

# check for the bounds of the world
minlat = None
minlon = None
maxlat = None
maxlon = None
with codecs.open(options.inFile, 'r', 'utf-8') as f:
    lines = f.read().splitlines()
    for line in lines:
        if 'bounds' in line:
            temp = float(re.findall(r'[-+]?\d*\.\d+|\d+', line[line.find('minlat'):])[0])
            if minlat is None or minlat > temp:
                minlat = temp
            temp = float(re.findall(r'[-+]?\d*\.\d+|\d+', line[line.find('minlon'):])[0])
            if minlon is None or minlon > temp:
                minlon = temp
            temp = float(re.findall(r'[-+]?\d*\.\d+|\d+', line[line.find('maxlat'):])[0])
            if maxlat is None or maxlat < temp:
                maxlat = temp
            temp = float(re.findall(r'[-+]?\d*\.\d+|\d+', line[line.find('maxlon'):])[0])
            if maxlon is None or maxlon < temp:
                maxlon = temp

if minlat is None or minlon is None or maxlat is None or maxlon is None:
    sys.stderr.write('Warning: impossible to get the map bounds from the OSM file,'
                     ' make sure the file contains the "bounds" tag.\n')
    sys.exit(0)

# Define the projection
lat0 = 0.5 * (maxlat + minlat)
long0 = 0.5 * (maxlon + minlon)
Projection.initProjection(long0, lat0, options.projection)

# Deal with the `extract-projection` argument
if options.extractProjection:
    print(Projection.getProjectionString())
    sys.exit()

# apply options to Webots objects
WebotsObject.enable3D = options.enable3D
WebotsObject.layerHeight = options.layer
WebotsObject.removalRadius = options.removalRadius
Road.noIntersectionRoadLines = options.noIntersectionRoadLines
Area.noForests = options.noForests

# get settings from config file
configFile = options.configFile
if not configFile:
    configFile = os.path.join(os.path.dirname(os.path.realpath(__file__)), "config.ini")
Settings.init(configFile)

# open output file (if it doesn't already exists)
if os.path.exists(options.outFile):
    sys.exit("Warning: file '" + options.outFile +
             "' already exists, remove it or change the destination using the '--output' option.\n")

with codecs.open(options.outFile, 'w', 'utf-8') as outputFile:
    # print headers and elevationGrid
    elevation = None
    if options.enable3D:
        elevation = Elevation(Projection.getProjection(), minlat=minlat, minlon=minlon, maxlat=maxlat, maxlon=maxlon,
                              googleAPIKey=options.googleAPIKey)
        print_header(options, outputFile, minlat=minlat, minlon=minlon, maxlat=maxlat, maxlon=maxlon, elevation=elevation)
        print(" * Elevation data acquired")
    else:
        print_header(options, outputFile, minlat=minlat, minlon=minlon, maxlat=maxlat, maxlon=maxlon)
    WebotsObject.elevation = elevation

    # parse OSM file
    parser = Parser()
    parser.parse_file(options.inFile, options.disableMultipolygonBuildings)
    Road.initialize_speed_limit(parser.country)

    print(" * OSM filed parsed")

    if options.enable3D and elevation is not None:
        add_height_to_coordinates(elevation)  # important to do it before 'center_coordinates'
    xOffset, yOffset = OSMCoord.center_coordinates(minlat=minlat, minlon=minlon, maxlat=maxlat, maxlon=maxlon)
    WebotsObject.xOffset = xOffset
    WebotsObject.yOffset = yOffset

    # From now we are in local coordinates system and not earth coordinates system anymore

    # print all the Webots objects
    if not options.noRoads:
        Road.process()
        Road.export(outputFile)
        print(" * " + str(len(Road.roads)) + " roads generated")
        print(" * " + str(len(Road.crossroads)) + " crossroads generated")
    if not options.noBuildings:
        Building.export(outputFile)
        print(" * " + str(len(Building.list)) + " buildings generated")
    if not options.noTrees:
        Tree.export(outputFile)
        print(" * " + str(len(Tree.list)) + " trees generated")
    if not options.noBarriers:
        Barrier.export(outputFile)
        print(" * " + str(len(Barrier.list)) + " barriers generated")
    if not options.noRivers:
        River.export(outputFile)
        print(" * " + str(len(River.list)) + " rivers generated")
    if not options.noAreas:
        Area.export(outputFile, options.noParkings)
        print(" * " + str(len(Area.list)) + " areas (forest, water, farmland, etc.) generated")
    if not options.noParkings:
        ParkingLines.export(outputFile)
        print(" * " + str(len(ParkingLines.list)) + " parking lines generated")

    print(" * map centered with this offset: " + str(xOffset) + "," + str(yOffset) + ").")
    print(" * reference coordinates: " + str(lat0) + "," + str(long0) + ".")
    print(" * projection used: '" + Projection.getProjectionString() + "'.")
    outputFile.close()
    print("Done.")
