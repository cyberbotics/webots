from controller import Supervisor

import c3d
import os.path
import sys

def isVirtualMarker(name):
    """Returns true is this is known to be a virtual marker."""
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
    """Gets a group of points and extract it's labels as a list of strings."""
    list = reader.groups['POINT'].get_string(name)
    elementSize = reader.groups['POINT'].get(name).dimensions[0]
    newlist = [list[i:i + elementSize] for i in range(0, len(list), elementSize)]
    for i in range(len(newlist)):
        newlist[i] = newlist[i].strip()
    return newlist

supervisor = Supervisor()
timestep = int(supervisor.getBasicTimeStep())

# parse arguments
if len(sys.argv) < 3:
    sys.exit('C3D file not defined.')

if not os.path.isfile(sys.argv[1]):
    sys.exit('\'%s\' does not exist.' % sys.argv[1])

playbackSpeed = float(sys.argv[2])

# parse C3D file
reader = c3d.Reader(open(sys.argv[1], 'rb'))

# extract point group labels
labels = getPointsList(reader, 'LABELS')
angleLabels = getPointsList(reader, 'ANGLES')
forcesLabels = getPointsList(reader, 'FORCES')
momentsLabels = getPointsList(reader, 'MOMENTS')
powersLabels = getPointsList(reader, 'POWERS')

# filter non 3D points and send the list to the robot window
filteredLabel = [x for x in labels if x not in angleLabels]
filteredLabel = [x for x in filteredLabel if x not in forcesLabels]
filteredLabel = [x for x in filteredLabel if x not in momentsLabels]
filteredLabel = [x for x in filteredLabel if x not in powersLabels]

markers = ''
virtualmarkers = ''
for label in filteredLabel:
    if isVirtualMarker(label):
        virtualmarkers += label + ' '
    else:
        markers += label + ' '
if markers:
    supervisor.wwiSendText('markers:' + markers.strip())
if virtualmarkers:
    supervisor.wwiSendText('virtual_markers:' + virtualmarkers.strip())

# get C3D files settings
numberOfpoints = reader.header.point_count
frameStep = 1.0 / reader.header.frame_rate
scale = reader.header.scale_factor
if reader.groups['POINT'].get('UNITS').string_value == 'mm':
    scale *= 0.001
elif not reader.groups['POINT'].get('UNITS').string_value == 'm':
    print("Can't determine the size unit.")

# make one step to be sure markers are not imported before pressing play
supervisor.step(timestep)

# remove possible previous marker (at regeneration for example)
markerField = supervisor.getSelf().getField('markers')
for i in range(markerField.getCount()):
    markerField.removeMF(-1)
# array = reader.groups['FORCE_PLATFORM'].get('CORNERS').float_array
# print(array)
# indexedFaceSet = "Shape {"
# indexedFaceSet += "geometry IndexedFaceSet {"
# indexedFaceSet += "coord Coordinate {"
# indexedFaceSet += "point ["
# for i in range(4):
#     indexedFaceSet += str(array[0][i][0] * scale) + " 0 " + str(array[2][i][0] * scale) +  ","
# indexedFaceSet += "]"
# indexedFaceSet += "}"
# indexedFaceSet += "coordIndex [0 1 2 3 -1]"
# indexedFaceSet += "}"
# indexedFaceSet += "}"
# markerField.importMFNodeFromString(-1, indexedFaceSet)

# import the marker and initize the list of points
pointRepresentations = {}
j = 0
for i in range(len(labels)):
    pointRepresentations[labels[i]] = {}
    pointRepresentations[labels[i]]['visible'] = False
    pointRepresentations[labels[i]]['node'] = None
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
for i, points, analog in reader.read_frames():
    frameAndPoints.append((i, points))

# main loop
frameCoutner = 0
totalFrameCoutner = 0
while supervisor.step(timestep) != -1:
    # check for messages from the robot-window
    message = supervisor.wwiReceiveText()
    while message:
        value = message.split(':')
        marker = value[0]
        action = value[1]
        if action == 'disable':
            pointRepresentations[marker]['visible'] = False
            pointRepresentations[marker]['transparency'].setSFFloat(1.0)
        elif action == 'enable':
            pointRepresentations[marker]['visible'] = True
            pointRepresentations[marker]['transparency'].setSFFloat(0.0)
        elif action == 'radius':
            pointRepresentations[marker]['radius'].setSFFloat(float(value[2]))
        elif action == 'color':
            h = value[2].lstrip('#')
            color = [int(h[i:i+2], 16) / 255 for i in (0, 2 ,4)]
            pointRepresentations[marker]['color'].setSFColor(color)
        message = supervisor.wwiReceiveText()

    # play the required frame (if needed)
    step = int(playbackSpeed * supervisor.getTime() / frameStep - totalFrameCoutner)
    if step > 0:
        frame = frameAndPoints[frameCoutner][0]
        points = frameAndPoints[frameCoutner][1]
        for j in range(numberOfpoints):
            if pointRepresentations[labels[j]]['visible']:
                x = points[j][0] * scale
                y = -points[j][2] * scale
                z = points[j][1] * scale
                pointRepresentations[labels[j]]['node'].getField('translation').setSFVec3f([x, y, z])
        totalFrameCoutner += step
        frameCoutner += step
        if frameCoutner >= len(frameAndPoints):
            frameCoutner = frameCoutner % len(frameAndPoints)
