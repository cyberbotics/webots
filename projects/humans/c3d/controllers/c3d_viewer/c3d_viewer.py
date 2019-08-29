# Copyright 1996-2019 Cyberbotics Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""C3D viewer controller."""

from controller import Supervisor

import c3d
import math
import os.path
import sys
transforms3dVailable = True
try:
    import transforms3d
except ImportError:
    transforms3dVailable = False


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


def getPointsList(reader, name):
    """Get a group of points and extract it's labels as a list of strings."""
    if name not in reader.groups['POINT'].params:
        return None
    list = reader.groups['POINT'].get_string(name)
    elementSize = reader.groups['POINT'].get(name).dimensions[0]
    newlist = [list[i:i + elementSize] for i in range(0, len(list), elementSize)]
    for i in range(len(newlist)):
        newlist[i] = newlist[i].strip()
    return newlist


supervisor = Supervisor()
timestep = int(supervisor.getBasicTimeStep())
enableMarkerGraph = False
enableValueGraphs = []

# parse arguments
if len(sys.argv) < 3:
    sys.exit('C3D file not defined.')

if not os.path.isfile(sys.argv[1]):
    sys.exit('\'%s\' does not exist.' % sys.argv[1])

playbackSpeed = float(sys.argv[2])

# parse C3D file
reader = c3d.Reader(open(sys.argv[1], 'rb'))

# get C3D files settings
numberOfpoints = reader.header.point_count
frameStep = 1.0 / reader.header.frame_rate
scale = -1.0 if reader.header.scale_factor < 0 else 1.0
if reader.groups['POINT'].get('UNITS').string_value.strip() == 'mm':
    scale *= 0.001
elif not reader.groups['POINT'].get('UNITS').string_value.strip() == 'm':
    print("Can't determine the size unit.")

# extract point group labels
labels = getPointsList(reader, 'LABELS')
angleLabels = getPointsList(reader, 'ANGLES')
forcesLabels = getPointsList(reader, 'FORCES')
momentsLabels = getPointsList(reader, 'MOMENTS')
powersLabels = getPointsList(reader, 'POWERS')

# get unit for each label group
pointGroup = reader.groups['POINT']
units = {
    'markers': 'm',
    'virtual_markers': 'm',
    'angles': pointGroup.get('ANGLE_UNITS').string_value if 'ANGLE_UNITS' in pointGroup.params else 'raw',
    'forces': pointGroup.get('FORCE_UNITS').string_value if 'FORCE_UNITS' in pointGroup.params else 'raw',
    'moments': pointGroup.get('MOMENT_UNITS').string_value if 'MOMENT_UNITS' in pointGroup.params else 'raw',
    'powers': pointGroup.get('POWER_UNITS').string_value if 'POWER_UNITS' in pointGroup.params else 'raw'
}

# filter non 3D points and send the list to the robot window
filteredLabel = labels[:numberOfpoints]
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
    if isVirtualMarker(label):
        virtualmarkers.append(label)
    else:
        markers.append(label)

# categorize each labels and send the lists to the robot window
labelsAndCategory = {
    'markers': markers,
    'virtual_markers': virtualmarkers,
    'angles': angleLabels,
    'forces': forcesLabels,
    'moments': momentsLabels,
    'powers': powersLabels
}
for key in labelsAndCategory:
    if labelsAndCategory[key]:
        supervisor.wwiSendText('labels:' + key + ':' + units[key] + ':' + ' '.join(labelsAndCategory[key]).strip())
supervisor.wwiSendText('configure:' + str(supervisor.getBasicTimeStep()))

# make one step to be sure markers are not imported before pressing play
supervisor.step(timestep)

# remove possible previous marker (at regeneration for example)
markerField = supervisor.getSelf().getField('markers')
for i in range(markerField.getCount()):
    markerField.removeMF(-1)

# ground reaction forces (GRF)
grfList = []
names = ['LGroundReaction', 'RGroundReaction']
for i in range(len(names)):
    if names[i] + 'Force' in labels and names[i] + 'Moment' in labels:
        grfList.append({'name': names[i]})
        markerField.importMFNodeFromString(-1, 'C3dGroundReactionForce { translation %s %s %s}' % (sys.argv[3 + 3 * i],
                                                                                                   sys.argv[4 + 3 * i],
                                                                                                   sys.argv[5 + 3 * i]))
        grfList[-1]['node'] = markerField.getMFNode(-1)
        grfList[-1]['rotation'] = grfList[-1]['node'].getField('rotation')
        grfList[-1]['cylinderTranslation'] = grfList[-1]['node'].getField('cylinderTranslation')
        grfList[-1]['coneTranslation'] = grfList[-1]['node'].getField('coneTranslation')
        grfList[-1]['height'] = grfList[-1]['node'].getField('height')

# get body visualization
bodyRotations = {}
bodyTranslations = {}
if float(sys.argv[9]) < 1.0:  # body transparency is not 1
    bodyNode = None
    height = float(sys.argv[10])
    if height < 0:
        if 'SUBJECTS' in reader.groups and reader.groups['SUBJECTS'].get('A_HEIGHT_MM') is not None:
            height = 0.001 * float(reader.groups['SUBJECTS'].get('A_HEIGHT_MM').string_value)
        elif 'SUBJECT' in reader.groups and reader.groups['SUBJECT'].get('HEIGHT') is not None:
            height = reader.groups['SUBJECT'].get('HEIGHT').float_value
        else:
            height = 1.83
    bodyScale = height / 1.83  # 1.83m: default size of the human model
    markerField.importMFNodeFromString(-1, 'DEF CentreOfMass_body C3dBodyRepresentation { transparency %s scale %lf %lf %lf }' %
                                       (sys.argv[9], bodyScale, bodyScale, bodyScale))
    bodyNode = markerField.getMFNode(-1)
    for label in labelsAndCategory['virtual_markers']:
        node = supervisor.getFromDef(label + '_body')
        if node:
            field = node.getField('translation')
            if field:
                bodyTranslations[label] = field
    if bodyNode and labelsAndCategory['angles'] is not None:
        for label in labelsAndCategory['angles']:
            field = bodyNode.getField(label.replace('Angles', 'Rotation'))
            if field:
                bodyRotations[label] = field

# import the marker and initialize the list of points
pointRepresentations = {}
j = 0
for i in range(len(labels)):
    pointRepresentations[labels[i]] = {}
    pointRepresentations[labels[i]]['visible'] = False
    pointRepresentations[labels[i]]['node'] = None
    pointRepresentations[labels[i]]['solid'] = supervisor.getFromDef(labels[i]) if labels[i] else None
    if labels[i] in filteredLabel:
        markerField.importMFNodeFromString(-1, 'C3dMarker { name "%s" }' % labels[i])
        pointRepresentations[labels[i]]['node'] = markerField.getMFNode(-1)
        pointRepresentations[labels[i]]['translation'] = pointRepresentations[labels[i]]['node'].getField('translation')
        pointRepresentations[labels[i]]['transparency'] = pointRepresentations[labels[i]]['node'].getField('transparency')
        pointRepresentations[labels[i]]['radius'] = pointRepresentations[labels[i]]['node'].getField('radius')
        pointRepresentations[labels[i]]['color'] = pointRepresentations[labels[i]]['node'].getField('color')
        if isVirtualMarker(labels[i]):
            pointRepresentations[labels[i]]['transparency'].setSFFloat(1.0)
        else:
            pointRepresentations[labels[i]]['visible'] = True
        j += 1

# parse the C3D frames
frameAndPoints = []
frameAndAnalog = []
for i, points, analog in reader.read_frames():
    frameAndPoints.append((i, points))

# main loop
frameCoutner = 0
totalFrameCoutner = 0
inverseY = reader.groups['POINT'].get('X_SCREEN').string_value.strip() == '+X'
while supervisor.step(timestep) != -1:
    # check for messages from the robot-window
    message = supervisor.wwiReceiveText()
    while message:
        value = message.split(':')
        action = value[0]
        if action == 'disable':
            for i in range(1, len(value)):
                pointRepresentations[value[i]]['visible'] = False
                pointRepresentations[value[i]]['transparency'].setSFFloat(1.0)
        elif action == 'enable':
            for i in range(1, len(value)):
                pointRepresentations[value[i]]['visible'] = True
                pointRepresentations[value[i]]['transparency'].setSFFloat(0.0)
        elif action == 'radius':
            for i in range(2, len(value)):
                pointRepresentations[value[i]]['radius'].setSFFloat(float(value[1]))
        elif action == 'color':
            h = value[1].lstrip('#')
            color = [int(h[i:i + 2], 16) / 255.0 for i in (0, 2, 4)]
            for i in range(2, len(value)):
                pointRepresentations[value[i]]['color'].setSFColor(color)
        elif action == 'graphs':
            if value[1] == 'markers':
                enableMarkerGraph = value[2] == 'true'
            else:
                if value[2] == 'true':
                    enableValueGraphs.append(value[1])
                else:
                    enableValueGraphs.remove(value[1])
        else:
            print(message)
        message = supervisor.wwiReceiveText()

    # play the required frame (if needed)
    step = int(playbackSpeed * supervisor.getTime() / frameStep - totalFrameCoutner)
    if step > 0:
        toSend = ''
        frame = frameAndPoints[frameCoutner][0]
        points = frameAndPoints[frameCoutner][1]
        # update the GRF visualization
        for grf in grfList:
            index1 = labels.index(grf['name'] + 'Force')
            index2 = labels.index(grf['name'] + 'Moment')
            COPX = 0
            COPY = 0
            if points[index1][3] >= 0:  # if index 3 or 4 is smaller than 0, the data is not valid for this frame
                COPX = 0.001 * points[index2][1] / points[index1][2]
                COPY = 0.001 * points[index2][0] / points[index1][2]
                vec1 = [0.0, 1, 0.0]
                vec2 = [0.04 * points[index1][0], 0.04 * points[index1][2], 0.04 * points[index1][1]]
                len1 = 1.0
                len2 = math.sqrt(math.pow(vec2[0], 2) + math.pow(vec2[1], 2) + math.pow(vec2[2], 2))
                vec3 = [0.0, 0.0, 0.0] if len2 == 0.0 else [vec2[0] / len2,  vec2[1] / len2, vec2[2] / len2]
                angle = math.acos(vec1[0] * vec3[0] + vec1[1] * vec3[1] + vec1[2] * vec3[2])  # acos(dot-product(vec1, vec3))
                axis = [vec1[1] * vec3[2] - vec1[2] * vec3[1],   # cross-product(vec1, vec3)
                        vec1[2] * vec3[0] - vec1[0] * vec3[2],
                        vec1[0] * vec3[1] - vec1[1] * vec3[0]]
                grf['cylinderTranslation'].setSFVec3f([COPX + 0.5 * vec2[0], 0.5 * vec2[1], COPY + 0.5 * vec2[2]])
                grf['coneTranslation'].setSFVec3f([COPX + 0.5 * vec2[0], 0.5 * (vec2[1] + len2), COPY + 0.5 * vec2[2]])
                if not (axis[0] == 0.0 and axis[1] == 0.0 and axis[2] == 0.0):
                    grf['rotation'].setSFRotation([axis[0], axis[1], axis[2], angle])
                grf['height'].setSFFloat(max(0.001, len2))
            else:
                grf['cylinderTranslation'].setSFVec3f([0, -1000, 0])
                grf['coneTranslation'].setSFVec3f([0, -1000, 0])
        # update the marker visualization
        for j in range(numberOfpoints):
            if pointRepresentations[labels[j]]['visible'] or pointRepresentations[labels[j]]['solid']:
                y = points[j][2] * scale
                if inverseY:
                    y = -y
                    x = points[j][0] * scale
                    z = -points[j][1] * scale
                else:
                    x = points[j][1] * scale
                    z = -points[j][0] * scale
                if pointRepresentations[labels[j]]['visible']:
                    pointRepresentations[labels[j]]['node'].getField('translation').setSFVec3f([x, y, z])
                    if enableMarkerGraph:
                        toSend += labels[j] + ':' + str(x) + ',' + str(y) + ',' + str(z) + ':'
                if pointRepresentations[labels[j]]['solid']:
                    pointRepresentations[labels[j]]['solid'].getField('translation').setSFVec3f([x, y, z])
            else:
                for categoryName in labelsAndCategory:
                    if labels[j] in labelsAndCategory[categoryName] and categoryName in enableValueGraphs:
                        x = points[j][0]
                        y = points[j][2]
                        z = points[j][1]
                        toSend += labels[j] + ':' + str(x) + ',' + str(y) + ',' + str(z) + ':'
            # update body representation (if any)
            if labels[j] in bodyRotations:
                if transforms3dVailable:
                    rot = transforms3d.euler.euler2axangle(angleSignAndOrder[labels[j]][0][0] * points[j][0] * math.pi / 180.0,
                                                           angleSignAndOrder[labels[j]][0][1] * points[j][1] * math.pi / 180.0,
                                                           angleSignAndOrder[labels[j]][0][2] * points[j][2] * math.pi / 180.0,
                                                           axes=angleSignAndOrder[labels[j]][1])
                    bodyRotations[labels[j]].setSFRotation([rot[0][0], rot[0][1], rot[0][2], rot[1]])
                else:
                    sys.stderr.write('Warning: "transforms3d" is required to update body representation.\n')
                    sys.stderr.write('Warning: You can install it with: "pip install transforms3d"\n')
            if labels[j] in bodyTranslations:
                y = points[j][2] * scale
                if inverseY:
                    y = -y
                    x = points[j][0] * scale
                    z = -points[j][1] * scale
                else:
                    x = points[j][1] * scale
                    z = -points[j][0] * scale
                bodyTranslations[labels[j]].setSFVec3f([x, y + float(sys.argv[11]), z])
        # send marker position to the robot window
        if toSend:
            toSend = toSend[:-1]  # remove last ':'
            supervisor.wwiSendText('positions:' + str(supervisor.getTime()) + ':' + toSend)
        totalFrameCoutner += step
        frameCoutner += step
        if frameCoutner >= len(frameAndPoints):
            frameCoutner = frameCoutner % len(frameAndPoints)
