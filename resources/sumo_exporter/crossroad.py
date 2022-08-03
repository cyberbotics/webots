"""Road class container."""

import math
import re

from re_definitions import floatRE, intRE
from data_structures import grouper
from lxml import etree as ET


class Crossroad(object):
    """Class matching with a Webots Crossroad, containing facilities to export to SUMO junctions."""

    crossroads = []

    def __init__(self, crossroadType):
        """Constructor: Initialize the crossroad with a unique id."""
        self.roads = []  # connected roads
        self.id = 'Custom%d' % len(Crossroad.crossroads)
        self.translation = [0.0, 0.0, 0.0]
        self.connectedRoadIDs = []
        self.shape = []
        self.crossroadType = crossroadType

    def init_from_wbt_string(self, wbtString):
        """Extract info from the wbtString matching the node."""
        try:
            self.id = re.findall(r'id\s*"([^"]*)"', wbtString)[0]
        except:
            pass
        try:
            self.translation = [float(x) for x in re.findall(r'translation\s*(%s\s*%s\s*%s)' %
                                                             (floatRE, floatRE, floatRE), wbtString)[0].split()]
        except:
            pass
        try:
            self.rotation = [float(x) for x in re.findall(r'rotation\s*(%s\s*%s\s*%s\s*%s)' %
                                                          (floatRE, floatRE, floatRE, floatRE), wbtString)[0].split()]
        except:
            self.rotation = [0.0, 0.0, 1.0, 0.0]
        try:
            self.connectedRoadIDs = [x.replace('"', '') for x in re.findall(r'connectedRoadIDs\s*\[([^\]]*)\]',
                                                                            wbtString)[0].split()]
        except:
            pass
        if self.crossroadType == "Crossroad":
            try:
                self.shape = grouper(3, [float(x) for x in re.findall(r'shape\s*\[([^\]]*)\]', wbtString)[0].split()])
                correction_angle = -math.pi * 0.5
                for i in range(len(self.shape)):
                    shape = self.shape[i]
                    x = - math.cos(correction_angle) * shape[0] + math.sin(correction_angle) * shape[1]
                    y = math.cos(correction_angle) * shape[1] + math.sin(correction_angle) * shape[0]
                    z = shape[2]
                    self.shape[i] = [x, y, z]
            except:
                pass
        elif self.crossroadType == "RoadIntersection":
            roadNumber = 4
            self.shape = []
            try:
                roadNumber = int(re.findall(r'roadNumber\s*(%s)' % intRE, wbtString)[0])
            except:
                roadNumber = 4
            roadsWidth = 7.0
            try:
                roadsWidth = float(re.findall(r'roadsWidth\s*(%s)' % floatRE, wbtString)[0])
            except:
                roadsWidth = 7.0
            outerRadius = roadsWidth / (2 * math.sin(math.pi / roadNumber))
            angle = -self.rotation[3]
            if self.rotation[2] > 0:
                angle = -angle
            for i in range(roadNumber):
                x1 = outerRadius * math.cos(2 * math.pi * i / roadNumber)
                y1 = outerRadius * math.sin(2 * math.pi * i / roadNumber)
                x2 = math.cos(angle) * x1 - math.sin(angle) * y1
                y2 = math.cos(angle) * y1 + math.sin(angle) * x1
                self.shape.append([x2, y2, 0])

    def create_node(self, nodes):
        """Populate the SUMO XML node."""
        node = ET.SubElement(nodes, 'node')
        node.attrib['id'] = self.id
        node.attrib['x'] = str(self.translation[0])
        node.attrib['y'] = str(self.translation[1])
        if self.shape:
            shape = ""
            for wayPoint in self.shape:
                shape += "%f,%f " % (wayPoint[0] + self.translation[0], wayPoint[1] + self.translation[1])
            shape += "%f,%f" % (self.shape[0][0] + self.translation[0], self.shape[0][1] + self.translation[1])
            node.attrib['shape'] = shape
