"""Road class container."""

import math
import re

from re_definitions import floatRE, intRE
from data_structures import grouper
from math_utils import apply_spline_subdivison_to_path
from shapely.geometry import LineString, MultiLineString
from lxml import etree as ET


class Road(object):
    """Class matching with a Webots Road, containing facilities to export to SUMO edges."""

    roads = []

    def __init__(self, wbtString, roadType):
        """Constructor: Extract info from the wbtString matching the node."""
        self.startJunction = None
        self.endJunction = None
        self.roadType = roadType

        try:
            self.id = re.findall(r'id\s*"([^"]*)"', wbtString)[0]
        except:
            self.id = ""
        try:
            self.width = float(re.findall(r'width\s*(%s)' % floatRE, wbtString)[0])
        except:
            self.width = 7
        try:
            self.speedLimit = float(re.findall(r'speedLimit\s*(%s)' % floatRE, wbtString)[0])
        except:
            self.speedLimit = 50.0 / 3.6  # 50 km/h
        try:
            self.translation = [float(x) for x in re.findall(r'translation\s*(%s\s*%s\s*%s)' % (floatRE, floatRE, floatRE), wbtString)[0].split()]
        except:
            self.translation = [0.0, 0.0, 0.0]
        try:
            self.rotation = [float(x) for x in re.findall(r'rotation\s*(%s\s*%s\s*%s\s*%s)' % (floatRE, floatRE, floatRE, floatRE), wbtString)[0].split()]
        except:
            self.rotation = [0.0, 1.0, 0.0, 0.0]
        try:
            self.startJunctionID = re.findall(r'startJunction\s*"([^"]*)"', wbtString)[0]
        except:
            self.startJunctionID = ""
        try:
            self.endJunctionID = re.findall(r'endJunction\s*"([^"]*)"', wbtString)[0]
        except:
            self.endJunctionID = ""
        if self.roadType == 'Road':
            try:
                self.wayPoints = grouper(3, [float(x) for x in re.findall(r'wayPoints\s*\[([^\]]*)\]', wbtString)[0].split()])
            except:
                self.wayPoints = []
            splineSubdivision = 4
            try:
                splineSubdivision = int(re.findall(r'splineSubdivision\s*(%s)' % intRE, wbtString)[0])
            except:
                splineSubdivision = 4
            if splineSubdivision > 0:
                self.wayPoints = apply_spline_subdivison_to_path(self.wayPoints, splineSubdivision)
        elif self.roadType == 'StraightRoadSegment':
            length = 10.0
            try:
                length = float(re.findall(r'length\s*(%s)' % floatRE, wbtString)[0])
            except:
                length = 10.0
            self.wayPoints = [[0, 0, 0], [0, 0, length]]
        elif self.roadType == 'CurvedRoadSegment':
            self.wayPoints = []
            subdivision = 8
            try:
                subdivision = int(re.findall(r'subdivision\s*(%s)' % intRE, wbtString)[0])
            except:
                subdivision = 8
            curvatureRadius = 10.0
            try:
                curvatureRadius = float(re.findall(r'curvatureRadius\s*(%s)' % floatRE, wbtString)[0])
            except:
                curvatureRadius = 10.0
            totalAngle = 1.5708
            try:
                totalAngle = float(re.findall(r'totalAngle\s*(%s)' % floatRE, wbtString)[0])
            except:
                totalAngle = 1.5708
            for i in range(subdivision + 1):
                x1 = curvatureRadius * math.cos(float(i) * totalAngle / float(subdivision))
                y1 = curvatureRadius * math.sin(float(i) * totalAngle / float(subdivision))
                self.wayPoints.append([x1, 0, y1])
        else:
            self.wayPoints = []
        try:
            self.lanes = int(re.findall(r'numberOfLanes\s*(%s)' % intRE, wbtString)[0])
        except:
            self.lanes = 2
        try:
            self.forwardLanes = int(re.findall(r'numberOfForwardLanes\s*(%s)' % intRE, wbtString)[0])
        except:
            self.forwardLanes = 1

        self.backwardLanes = self.lanes - self.forwardLanes
        self.oneWay = self.backwardLanes == 0

        if self.rotation[0] < 0.01 and self.rotation[2] < 0.01:
            angle = self.rotation[3]
            if self.rotation[1] > 0:
                angle = -angle
            for i in range(len(self.wayPoints)):
                wayPoint = self.wayPoints[i]
                x = math.cos(angle) * wayPoint[0] - math.sin(angle) * wayPoint[2]
                y = wayPoint[1]
                z = math.cos(angle) * wayPoint[2] + math.sin(angle) * wayPoint[0]
                self.wayPoints[i] = [x, y, z]
        else:
            print ('Warning: cannot export edge "%s" because the road is rotated not only along axis Y.' % self.id)

    def create_edge(self, edges):
        """Create the SUMO edge XML node(s) matching with the Webots road."""
        if self.startJunctionID == self.endJunctionID:
            print ('Warning: cannot export edge "%s" because start and end junctions are identical.' % self.id)
            return

        if len(self.wayPoints) < 2:
            print ('Warning: cannot export edge "%s" because it has less than 2 way-points.' % self.id)
            return

        laneWidth = self.width / self.lanes

        # The original path should be slightly shifted if the case where the
        # forwardLanes and backwardLanes are not matching.
        originalCoords = [[- x - self.translation[0], z + self.translation[2]] for [x, y, z] in self.wayPoints]
        originalLineString = LineString(originalCoords)
        if self.oneWay:
            originalLineString = originalLineString.parallel_offset(0.5 * laneWidth * self.forwardLanes, 'left')
        else:
            offset = (self.forwardLanes - self.backwardLanes) * laneWidth * 0.5
            if offset > 0.0:
                originalLineString = originalLineString.parallel_offset(offset, 'left')
            elif offset < 0.0:
                originalLineString = originalLineString.parallel_offset(offset, 'left')
                originalLineString = LineString(list(originalLineString.coords[::-1]))

        if isinstance(originalLineString, MultiLineString):
            originalPath = originalCoords
        else:
            originalPath = list(originalLineString.coords)

        # Create the forward edge
        if self.forwardLanes > 0:
            edge = ET.SubElement(edges, 'edge')
            edge.attrib['id'] = self.id
            edge.attrib['from'] = self.startJunctionID
            edge.attrib['to'] = self.endJunctionID
            edge.attrib['numLanes'] = str(self.forwardLanes)
            edge.attrib['width'] = str(laneWidth)
            edge.attrib['shape'] = Road._pathToString(originalPath)
        # Create the backward edge
        if self.backwardLanes > 0:
            edge = ET.SubElement(edges, 'edge')
            edge.attrib['id'] = '-' + self.id
            edge.attrib['to'] = self.startJunctionID
            edge.attrib['from'] = self.endJunctionID
            edge.attrib['numLanes'] = str(self.backwardLanes)
            edge.attrib['width'] = str(laneWidth)
            edge.attrib['shape'] = Road._pathToString(originalPath[::-1])

    @classmethod
    def _pathToString(cls, path):
        s = ""
        for coord in path:
            s += "%f,%f " % (coord[0], coord[1])
        return s
