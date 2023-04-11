"""Export a SUMO network file starting from Webots roads."""

import optparse
import os
import sys

from lxml import etree as ET

from crossroad import Crossroad
from node_extractor import NodeExtractor
from road import Road


# Parse the options.
optParser = optparse.OptionParser(usage="usage: %prog --input=file.wbt --output=.")
optParser.add_option("--input", dest="input", default="file.wbt", help="specifies the Webots .wbt file to open.")
optParser.add_option("--output", dest="output", default=".",
                     help="specifies the directory where to generate the SUMO network files.")
options, args = optParser.parse_args()

# Check options.
if not options.input.endswith('.wbt'):
    sys.exit('Invalid input file format.')
if not os.path.exists(options.input):
    sys.exit('Input file does not exists.')
if not os.path.isdir(options.output):
    sys.exit('Output directory does not exist. Please create it.')

# Compute the output file paths.
nodesFilePath = os.path.join(options.output, 'sumo.nod.xml')
edgesFilePath = os.path.join(options.output, 'sumo.edg.xml')
configFilePath = os.path.join(options.output, 'sumo.sumocfg')

# Check these files are not existing.
if os.path.exists(nodesFilePath):
    sys.exit('"%s" already exists. Please remove it first.' % nodesFilePath)
if os.path.exists(edgesFilePath):
    sys.exit('"%s" already exists. Please remove it first.' % edgesFilePath)
if os.path.exists(configFilePath):
    sys.exit('"%s" already exists. Please remove it first.' % configFilePath)

# Extracts map info, roads and crossroads from the Webots file
nodeExtractor = NodeExtractor(options.input)

roadTypes = ["Road", "StraightRoadSegment", "CurvedRoadSegment"]
for roadType in roadTypes:
    for roadString in nodeExtractor.extractRootNodes(roadType):
        road = Road(roadString, roadType)
        Road.roads.append(road)
crossroadTypes = ["Crossroad", "RoadIntersection", "LaneSeparation"]
for crossroadType in crossroadTypes:
    for crossroadString in nodeExtractor.extractRootNodes(crossroadType):
        crossroad = Crossroad(crossroadType)
        crossroad.init_from_wbt_string(crossroadString)
        Crossroad.crossroads.append(crossroad)

# Link the Crossroad to the Road objects
for crossroad in Crossroad.crossroads:
    crossroad.roads = []
    for roadID in crossroad.connectedRoadIDs:
        for road in Road.roads:
            if road.id == roadID:
                crossroad.roads.append(road)
for road in Road.roads:
    for crossroad in Crossroad.crossroads:
        if road.startJunctionID == crossroad.id:
            road.startJunction = crossroad
        if road.endJunctionID == crossroad.id:
            road.endJunction = crossroad
    # Fill the extremities of the Roads having no crossroad
    # with empty crossroads to let work SUMO.
    if road.startJunction is None and len(road.wayPoints) > 1:
        crossroad = Crossroad("Crossroad")
        crossroad.connectedRoadIDs.append(road.id)
        crossroad.translation = [
            road.wayPoints[0][0] + road.translation[0],
            road.wayPoints[0][1] + road.translation[1],
            road.wayPoints[0][2] + road.translation[2]
        ]
        Crossroad.crossroads.append(crossroad)
        road.startJunction = crossroad
        road.startJunctionID = crossroad.id
    if road.endJunction is None and len(road.wayPoints) > 1:
        crossroad = Crossroad("Crossroad")
        crossroad.connectedRoadIDs.append(road.id)
        crossroad.translation = [
            road.wayPoints[-1][0] + road.translation[0],
            road.wayPoints[-1][1] + road.translation[1],
            road.wayPoints[-1][2] + road.translation[2]
        ]
        Crossroad.crossroads.append(crossroad)
        road.endJunction = crossroad
        road.endJunctionID = crossroad.id

# Export to SUMO nodes file
nodes = ET.Element('nodes')
for crossroad in Crossroad.crossroads:
    crossroad.create_node(nodes)
tree = ET.ElementTree(nodes)
tree.write(nodesFilePath, encoding='utf-8', xml_declaration=True, pretty_print=True)

# Export to SUMO edges file
edges = ET.Element('edges')
for road in Road.roads:
    road.create_edge(edges)
tree = ET.ElementTree(edges)
tree.write(edgesFilePath, encoding='utf-8', xml_declaration=True, pretty_print=True)

# Export SUMO configuration file
configuration = ET.Element('configuration')
tree = ET.ElementTree(configuration)
input = ET.SubElement(configuration, 'input')
ET.SubElement(input, 'net-file', value='sumo.net.xml')
ET.SubElement(input, 'route-files', value='sumo.rou.xml')
time = ET.SubElement(configuration, 'time')
ET.SubElement(time, 'begin', value='0')
report = ET.SubElement(configuration, 'report')
ET.SubElement(report, 'verbose', value='true')
ET.SubElement(report, 'no-step-log', value='true')
tree.write(configFilePath, encoding='utf-8', xml_declaration=True, pretty_print=True)
