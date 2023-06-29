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

"""C3D viewer controller."""

from controller import Supervisor

import base64
import c3d
import math
import os.path
import sys
import tempfile

transforms3dAvailable = True
try:
    import transforms3d
except ImportError:
    transforms3dAvailable = False


angleSignAndOrder = {
    'RShoulderAngles': ((-1, -1, 1), 'rxzy'),
    'RElbowAngles': ((-1, 0, 0), 'rxzy'),
    'RWristAngles': ((1, -1, -1), 'rxzy'),
    'LShoulderAngles': ((-1, 1, -1), 'rxzy'),
    'LElbowAngles': ((-1, 0, 0), 'rxzy'),
    'LWristAngles': ((-1, 1, 1), 'rxzy'),
    'LHipAngles': ((-1, -1, -1), 'rxzy'),
    'LKneeAngles': ((1, -1, -1), 'rxzy'),
    'LAnkleAngles': ((-1, -1, -1), 'rxyz'),
    'RHipAngles': ((-1, 1, 1), 'rxzy'),
    'RKneeAngles': ((1, 1, 1), 'rxzy'),
    'RAnkleAngles': ((-1, 1, 1), 'rxyz'),
    'LPelvisAngles': ((1, 1, -1), 'rxzy')
}


class c3dFile:
    """This class holds a C3D file and performs the necessary setup to run it."""
    """In particular, it inserts and deletes 3D markers in the scene tree and communicates with the robot window."""
    def __init__(self, name):
        self.x = 0
        if not os.path.isfile(name):
            sys.exit('\'%s\' does not exist.' % name)
        # parse C3D file
        with open(name, 'rb') as file:
            self.reader = c3d.Reader(file)

            # get C3D files settings
            self.numberOfPoints = self.reader.header.point_count
            self.frameStep = 1.0 / self.reader.header.frame_rate
            self.scale = -1.0 if self.reader.header.scale_factor < 0 else 1.0
            if self.reader.groups['POINT'].get('UNITS').string_value.strip() == 'mm':
                self.scale *= 0.001
            elif not self.reader.groups['POINT'].get('UNITS').string_value.strip() == 'm':
                print("Can't determine the size unit.")

            # extract point group labels
            self.labels = self.getPointsList('LABELS') if self.getPointsList('LABELS') is not None else []
            angleLabels = self.getPointsList('ANGLES') if self.getPointsList('ANGLES') is not None else []
            forcesLabels = self.getPointsList('FORCES') if self.getPointsList('FORCES') is not None else []
            momentsLabels = self.getPointsList('MOMENTS') if self.getPointsList('MOMENTS') is not None else []
            powersLabels = self.getPointsList('POWERS') if self.getPointsList('POWERS') is not None else []

            # get unit for each label group
            pointGroup = self.reader.groups['POINT']
            units = {
                'markers': 'm',
                'virtual_markers': 'm',
                'angles': pointGroup.get('ANGLE_UNITS').string_value if 'ANGLE_UNITS' in pointGroup.params else 'raw',
                'forces': pointGroup.get('FORCE_UNITS').string_value if 'FORCE_UNITS' in pointGroup.params else 'raw',
                'moments': pointGroup.get('MOMENT_UNITS').string_value if 'MOMENT_UNITS' in pointGroup.params else 'raw',
                'powers': pointGroup.get('POWER_UNITS').string_value if 'POWER_UNITS' in pointGroup.params else 'raw'
            }

            # filter non 3D points and send the list to the robot window
            filteredLabel = self.labels[:self.numberOfPoints]
            if angleLabels:
                filteredLabel = [x for x in filteredLabel if x not in angleLabels]
            if forcesLabels:
                filteredLabel = [x for x in filteredLabel if x not in forcesLabels]
            if momentsLabels:
                filteredLabel = [x for x in filteredLabel if x not in momentsLabels]
            if powersLabels:
                filteredLabel = [x for x in filteredLabel if x not in powersLabels]

            # split between actual and virtual markers
            markers = []
            virtualmarkers = []
            for label in filteredLabel:
                if self.isVirtualMarker(label):
                    virtualmarkers.append(label)
                else:
                    markers.append(label)

            # categorize each labels and send the lists to the robot window
            self.labelsAndCategory = {
                'markers': markers,
                'virtual_markers': virtualmarkers,
                'angles': angleLabels,
                'forces': forcesLabels,
                'moments': momentsLabels,
                'powers': powersLabels
            }
            for key in self.labelsAndCategory:
                if self.labelsAndCategory[key]:
                    supervisor.wwiSendText('labels:' + key + ':' + units[key] + ':' +
                                           ' '.join(self.labelsAndCategory[key]).strip())
                else:
                    supervisor.wwiSendText('labels:' + key + ':' + units[key] + ':' + 'None')
            supervisor.wwiSendText('configure:' + str(supervisor.getBasicTimeStep()))
            # ground reaction forces (GRF)
            self.grfList = []
            names = ['LGroundReaction', 'RGroundReaction']
            markerField = supervisor.getSelf().getField('markers')

            for i in range(len(names)):
                if names[i] + 'Force' in self.labels and names[i] + 'Moment' in self.labels:
                    self.grfList.append({'name': names[i]})
                    markerField.importMFNodeFromString(-1, 'C3dGroundReactionForce { translation %s %s %s}' %
                                                           (sys.argv[3 + 3 * i],
                                                            sys.argv[4 + 3 * i],
                                                            sys.argv[5 + 3 * i]))
                    self.grfList[-1]['node'] = markerField.getMFNode(-1)
                    self.grfList[-1]['rotation'] = self.grfList[-1]['node'].getField('rotation')
                    self.grfList[-1]['cylinderTranslation'] = self.grfList[-1]['node'].getField('cylinderTranslation')
                    self.grfList[-1]['coneTranslation'] = self.grfList[-1]['node'].getField('coneTranslation')
                    self.grfList[-1]['height'] = self.grfList[-1]['node'].getField('height')

            # get body visualization
            self.bodyRotations = {}
            self.bodyTranslations = {}
            bodyNode = None
            self.bodyTransparency = float(sys.argv[9])
            self.height = float(sys.argv[10])
            if self.height < 0:
                if 'SUBJECTS' in self.reader.groups and self.reader.groups['SUBJECTS'].get('A_HEIGHT_MM') is not None:
                    self.height = 0.001 * float(self.reader.groups['SUBJECTS'].get('A_HEIGHT_MM').string_value)
                elif 'SUBJECT' in self.reader.groups and self.reader.groups['SUBJECT'].get('HEIGHT') is not None:
                    self.height = self.reader.groups['SUBJECT'].get('HEIGHT').float_value
                else:
                    self.height = 1.83
            bodyScale = self.height / 1.83  # 1.83m: default size of the human model
            markerField.importMFNodeFromString(-1,
                                               'DEF CentreOfMass_body C3dBodyRepresentation '
                                               '{ transparency %s scale %lf %lf %lf }' %
                                               (self.bodyTransparency, bodyScale, bodyScale, bodyScale))
            bodyNode = markerField.getMFNode(-1)
            self.bodyTransparencyField = bodyNode.getField('transparency')
            for label in self.labelsAndCategory['virtual_markers']:
                node = supervisor.getFromDef(label + '_body')
                if node:
                    field = node.getField('translation')
                    if field:
                        self.bodyTranslations[label] = field
            if bodyNode and self.labelsAndCategory['angles'] is not None:
                for label in self.labelsAndCategory['angles']:
                    field = bodyNode.getField(label.replace('Angles', 'Rotation'))
                    if field:
                        self.bodyRotations[label] = field

            # import the marker and initialize the list of points
            self.pointRepresentations = {}
            j = 0
            for i in range(len(self.labels)):
                label = self.labels[i]
                pointRepresentation = {}
                pointRepresentation['visible'] = False
                pointRepresentation['node'] = None
                pointRepresentation['solid'] = supervisor.getFromDef(label) if label else None
                if label in filteredLabel:
                    markerField.importMFNodeFromString(-1, 'C3dMarker { name "%s" }' % label)
                    pointRepresentation['node'] = markerField.getMFNode(-1)
                    pointRepresentation['translation'] = pointRepresentation['node'].getField('translation')
                    pointRepresentation['transparency'] = pointRepresentation['node'].getField('transparency')
                    pointRepresentation['radius'] = pointRepresentation['node'].getField('radius')
                    pointRepresentation['color'] = pointRepresentation['node'].getField('color')
                    if self.isVirtualMarker(label):
                        pointRepresentation['transparency'].setSFFloat(1.0)
                    else:
                        pointRepresentation['visible'] = True
                    j += 1
                self.pointRepresentations[label] = pointRepresentation

            # parse the C3D frames
            self.frameAndPoints = []
            self.frameAndAnalog = []
            for i, points, analog in self.reader.read_frames():
                self.frameAndPoints.append((i, points))

            X_SCREEN = self.reader.groups['POINT'].get('X_SCREEN').string_value.strip()
            Y_SCREEN = self.reader.groups['POINT'].get('Y_SCREEN').string_value.strip()
            self.inverseY = (X_SCREEN == '+X' and Y_SCREEN == '+Z') or (X_SCREEN == '-X' and Y_SCREEN == '-Z')

    def __del__(self):
        c3dFile.removeMarkers()
        supervisor.wwiSendText('reset')

    def getPointsList(self, name):
        """Get a group of points and extract it's labels as a list of strings."""
        if name not in self.reader.groups['POINT'].params:
            return None
        list = self.reader.groups['POINT'].get_string(name)
        elementSize = self.reader.groups['POINT'].get(name).dimensions[0]
        newlist = [list[i:i + elementSize] for i in range(0, len(list), elementSize)]
        for i in range(len(newlist)):
            newlist[i] = newlist[i].strip()
        return newlist

    @staticmethod
    def isVirtualMarker(name):
        """Return true is this is known to be a virtual marker."""
        prefixList = [
            'HED', 'LCL', 'LFE', 'LFO', 'LHN', 'LHU', 'LRA', 'LTI', 'LTO', 'PEL',
            'RCL', 'RFE', 'RFO', 'RHN', 'RHU', 'RRA', 'RTI', 'RTO', 'TRX'
        ]
        virtualMarkers = ['CentreOfMass', 'CentreOfMassFloor']
        for prefix in prefixList:
            virtualMarkers.append(prefix + 'O')
            virtualMarkers.append(prefix + 'A')
            virtualMarkers.append(prefix + 'L')
            virtualMarkers.append(prefix + 'P')
        if name in virtualMarkers:
            return True
        return False

    @staticmethod
    def removeMarkers():
        markerField = supervisor.getSelf().getField('markers')
        for i in range(markerField.getCount()):
            markerField.removeMF(-1)


supervisor = Supervisor()
timestep = int(supervisor.getBasicTimeStep())
enableValueGraphs = []

# parse arguments
if len(sys.argv) < 3 or sys.argv[1] == 'None':
    sys.exit('C3D file not defined.')

playbackSpeed = float(sys.argv[2])

# make one step to be sure markers are not imported before pressing play
supervisor.step(timestep)

# remove possible previous marker (at regeneration for example)
c3dFile.removeMarkers()

c3dfile = c3dFile(sys.argv[1])

# main loop
frameCoutner = 0
totalFrameCoutner = 0
offsetTime = 0

while supervisor.step(timestep) != -1:
    # check for messages from the robot-window
    message = supervisor.wwiReceiveText()
    while message:
        value = message.split(':')
        action = value[0]
        if action == 'disable':
            for i in range(1, len(value)):
                c3dfile.pointRepresentations[value[i]]['visible'] = False
                c3dfile.pointRepresentations[value[i]]['transparency'].setSFFloat(1.0)
        elif action == 'enable':
            for i in range(1, len(value)):
                c3dfile.pointRepresentations[value[i]]['visible'] = True
                c3dfile.pointRepresentations[value[i]]['transparency'].setSFFloat(0.0)
        elif action == 'radius':
            for i in range(2, len(value)):
                c3dfile.pointRepresentations[value[i]]['radius'].setSFFloat(float(value[1]))
        elif action == 'color':
            h = value[1].lstrip('#')
            color = [int(h[i:i + 2], 16) / 255.0 for i in (0, 2, 4)]
            for i in range(2, len(value)):
                c3dfile.pointRepresentations[value[i]]['color'].setSFColor(color)
        elif action == 'graphs':
            if value[2] == 'true':
                enableValueGraphs.append(value[1])
            else:
                enableValueGraphs.remove(value[1])
        elif action == 'body_transparency':
            c3dfile.bodyTransparency = float(value[1])
            c3dfile.bodyTransparencyField.setSFFloat(c3dfile.bodyTransparency)
        elif action == 'speed':
            playbackSpeed = float(value[1])
            offsetTime = supervisor.getTime()
            totalFrameCoutner = 0
        elif action == 'c3dfile':
            file = tempfile.NamedTemporaryFile(mode='wb', delete=False, suffix='.c3d')
            file.write(base64.b64decode(value[1]))
            file.close()
            del c3dfile
            c3dfile = c3dFile(file.name)
            os.remove(file.name)
        else:
            print(message)

        message = supervisor.wwiReceiveText()

    # play the required frame (if needed)
    step = int(playbackSpeed * (supervisor.getTime() - offsetTime) / c3dfile.frameStep - totalFrameCoutner)
    if step > 0:
        toSend = ''
        frame = c3dfile.frameAndPoints[frameCoutner][0]
        points = c3dfile.frameAndPoints[frameCoutner][1]
        # update the GRF visualization
        for grf in c3dfile.grfList:
            index1 = c3dfile.labels.index(grf['name'] + 'Force')
            index2 = c3dfile.labels.index(grf['name'] + 'Moment')
            CenterOfPressureX = 0
            CenterOfPressureY = 0
            if points[index1][3] >= 0:  # if index 3 or 4 is smaller than 0, the data is not valid for this frame
                CenterOfPressureX = 0.001 * points[index2][1] / points[index1][2]
                CenterOfPressureY = 0.001 * points[index2][0] / points[index1][2]
                vec1 = [0.0, 1, 0.0]
                vec2 = [0.04 * points[index1][0], 0.04 * points[index1][2], 0.04 * points[index1][1]]
                len1 = 1.0
                len2 = math.sqrt(math.pow(vec2[0], 2) + math.pow(vec2[1], 2) + math.pow(vec2[2], 2))
                vec3 = [0.0, 0.0, 0.0] if len2 == 0.0 else [vec2[0] / len2,  vec2[1] / len2, vec2[2] / len2]
                angle = math.acos(vec1[0] * vec3[0] + vec1[1] * vec3[1] + vec1[2] * vec3[2])  # acos(dot-product(vec1, vec3))
                axis = [vec1[1] * vec3[2] - vec1[2] * vec3[1],   # cross-product(vec1, vec3)
                        vec1[2] * vec3[0] - vec1[0] * vec3[2],
                        vec1[0] * vec3[1] - vec1[1] * vec3[0]]
                grf['cylinderTranslation'].setSFVec3f([CenterOfPressureX + 0.5 * vec2[0],
                                                       CenterOfPressureY + 0.5 * vec2[2],
                                                       0.5 * vec2[1]])
                grf['coneTranslation'].setSFVec3f([CenterOfPressureX + 0.5 * vec2[0],
                                                   CenterOfPressureY + 0.5 * vec2[2],
                                                   0.5 * (vec2[1] + len2)])
                if not (axis[0] == 0.0 and axis[1] == 0.0 and axis[2] == 0.0):
                    grf['rotation'].setSFRotation([axis[0], axis[1], axis[2], angle])
                grf['height'].setSFFloat(max(0.001, len2))
            else:
                grf['cylinderTranslation'].setSFVec3f([0, 0, -1000])
                grf['coneTranslation'].setSFVec3f([0, 0, -1000])
        # update the markers visualization and graph and body angles
        for j in range(c3dfile.numberOfPoints):
            # get actual coordinates
            if c3dfile.inverseY:
                x = points[j][0] * c3dfile.scale
                y = -points[j][1] * c3dfile.scale
                z = -points[j][2] * c3dfile.scale
            else:
                x = points[j][1] * c3dfile.scale
                y = -points[j][0] * c3dfile.scale
                z = points[j][2] * c3dfile.scale
            # update markers visualization
            label = c3dfile.labels[j]
            if c3dfile.pointRepresentations[label]['visible'] or c3dfile.pointRepresentations[label]['solid']:
                if c3dfile.pointRepresentations[label]['visible']:
                    c3dfile.pointRepresentations[label]['node'].getField('translation').setSFVec3f([x, y, z])
                if c3dfile.pointRepresentations[label]['solid']:
                    c3dfile.pointRepresentations[label]['solid'].getField('translation').setSFVec3f([x, y, z])
            # update markers graph
            for categoryName in c3dfile.labelsAndCategory:
                if label in c3dfile.labelsAndCategory[categoryName] and categoryName in enableValueGraphs:
                    if categoryName in ['markers', 'virtual_markes']:
                        toSend += label + ':' + str(x) + ',' + str(y) + ',' + str(z) + ':'
                    else:
                        toSend += label + ':' + str(points[j][0]) + ',' + str(points[j][1]) + ',' + str(points[j][2]) + ':'
            # update body representation (if any)
            if label in c3dfile.bodyRotations and c3dfile.bodyTransparency < 1.0:
                if transforms3dAvailable:
                    rot = transforms3d.euler.euler2axangle(angleSignAndOrder[label][0][0] * points[j][0] * math.pi / 180.0,
                                                           angleSignAndOrder[label][0][1] * points[j][1] * math.pi / 180.0,
                                                           angleSignAndOrder[label][0][2] * points[j][2] * math.pi / 180.0,
                                                           axes=angleSignAndOrder[label][1])
                    c3dfile.bodyRotations[label].setSFRotation([rot[0][0], rot[0][1], rot[0][2], rot[1]])
                else:
                    sys.stderr.write('Warning: "transforms3d" is required to update body representation.\n')
                    sys.stderr.write('Warning: You can install it with: "pip install transforms3d"\n')
            if label in c3dfile.bodyTranslations:
                c3dfile.bodyTranslations[label].setSFVec3f([x, y + float(sys.argv[11]), z])
        # send marker position to the robot window
        if toSend:
            toSend = toSend[:-1]  # remove last ':'
            supervisor.wwiSendText('positions:' + str(supervisor.getTime()) + ':' + toSend)
        totalFrameCoutner += step
        frameCoutner += step
        if frameCoutner >= len(c3dfile.frameAndPoints):
            frameCoutner = frameCoutner % len(c3dfile.frameAndPoints)
