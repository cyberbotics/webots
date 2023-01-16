"""Road class container."""

import math
import re

from re_definitions import floatRE, intRE
from data_structures import grouper
from math_utils import apply_spline_subdivision_to_path
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

        id = re.findall(r'id\s*"([^"]*)"', wbtString)
        self.id = id[0] if id else ""

        width = re.findall(r'width\s*(%s)' % floatRE, wbtString)
        self.width = float(width[0]) if width else 7

        speedLimit = re.findall(r'speedLimit\s*(%s)' % floatRE, wbtString)
        self.speedLimit = float(speedLimit[0]) if speedLimit else 50.0 / 3.6  # 50 km/h

        translation = re.findall(r'translation\s*(%s\s*%s\s*%s)' % (floatRE, floatRE, floatRE), wbtString)
        self.translation = [float(x) for x in translation[0].split()] if translation else [0.0, 0.0, 0.0]

        rotation = re.findall(r'rotation\s*(%s\s*%s\s*%s\s*%s)' % (floatRE, floatRE, floatRE, floatRE), wbtString)
        self.rotation = [float(x) for x in rotation[0].split()] if rotation else [0.0, 0.0, 1.0, 0]

        startJunctionID = re.findall(r'startJunction\s*"([^"]*)"', wbtString)
        self.startJunctionID = startJunctionID[0] if startJunctionID else ""

        endJunctionID = re.findall(r'endJunction\s*"([^"]*)"', wbtString)
        self.endJunctionID = endJunctionID[0] if endJunctionID else ""

        if self.roadType == 'Road':
            try:
                self.wayPoints = grouper(3, [float(x) for x in re.findall(r'wayPoints\s*\[([^\]]*)\]', wbtString)[0].split()])
                correction_angle = math.pi * 0.5
                for i in range(len(self.wayPoints)):
                    wayPoint = self.wayPoints[i]
                    x = -math.cos(correction_angle) * wayPoint[0] + math.sin(correction_angle) * wayPoint[1]
                    y = math.cos(correction_angle) * wayPoint[1] + math.sin(correction_angle) * wayPoint[0]
                    z = wayPoint[2]
                    self.wayPoints[i] = [x, y, z]
            except Exception:
                self.wayPoints = []

            splineSubdivision = re.findall(r'splineSubdivision\s*(%s)' % intRE, wbtString)
            splineSubdivision = int(splineSubdivision[0]) if splineSubdivision else 4
            if splineSubdivision > 0:
                self.wayPoints = apply_spline_subdivision_to_path(self.wayPoints, splineSubdivision)
        elif self.roadType == 'StraightRoadSegment':
            length = re.findall(r'length\s*(%s)' % floatRE, wbtString)
            length = float(length[0]) if length else 10.0
            self.wayPoints = [[0, 0, 0], [0, length, 0]]
        elif self.roadType == 'CurvedRoadSegment':
            self.wayPoints = []

            subdivision = re.findall(r'subdivision\s*(%s)' % intRE, wbtString)
            subdivision = int(subdivision[0]) if subdivision else 16

            curvatureRadius = re.findall(r'curvatureRadius\s*(%s)' % floatRE, wbtString)
            curvatureRadius = float(curvatureRadius[0]) if curvatureRadius else 10.0

            totalAngle = re.findall(r'totalAngle\s*(%s)' % floatRE, wbtString)
            totalAngle = float(totalAngle[0]) if totalAngle else 1.5708

            for i in range(subdivision + 1):
                x1 = curvatureRadius * math.cos(float(i) * totalAngle / float(subdivision))
                y1 = curvatureRadius * math.sin(float(i) * totalAngle / float(subdivision))
                self.wayPoints.append([x1, y1, 0])
        else:
            self.wayPoints = []

        lanes = re.findall(r'numberOfLanes\s*(%s)' % intRE, wbtString)
        self.lanes = int(lanes[0]) if lanes else 2

        forwardLanes = re.findall(r'numberOfForwardLanes\s*(%s)' % intRE, wbtString)[0]
        self.forwardLanes = int(forwardLanes[0]) if forwardLanes else 1

        self.backwardLanes = self.lanes - self.forwardLanes
        self.oneWay = self.backwardLanes == 0

        if self.rotation[0] < 0.01 and self.rotation[1] < 0.01:
            angle = self.rotation[3]
            if self.rotation[2] < 0:
                angle = -angle
            for i in range(len(self.wayPoints)):
                wayPoint = self.wayPoints[i]
                x = math.cos(angle) * wayPoint[1] - math.sin(angle) * wayPoint[0]
                y = math.cos(angle) * wayPoint[0] + math.sin(angle) * wayPoint[1]
                z = wayPoint[2]
                self.wayPoints[i] = [x, y, z]
        else:
            print('Warning: cannot export edge "%s" because the road is rotated not only along axis Z.' % self.id)

    def create_edge(self, edges):
        """Create the SUMO edge XML node(s) matching with the Webots road."""
        if self.startJunctionID == self.endJunctionID:
            print('Warning: cannot export edge "%s" because start and end junctions are identical.' % self.id)
            return

        if len(self.wayPoints) < 2:
            print('Warning: cannot export edge "%s" because it has less than 2 way-points.' % self.id)
            return

        laneWidth = self.width / self.lanes

        # The original path should be slightly shifted if the case where the
        # forwardLanes and backwardLanes are not matching.
        originalCoords = [[x + self.translation[0], y + self.translation[1]] for [x, y, z] in self.wayPoints]
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
