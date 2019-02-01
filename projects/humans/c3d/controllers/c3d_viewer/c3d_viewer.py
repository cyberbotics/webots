from controller import Supervisor

import c3d
import os.path
import sys

def getPointsList(reader, name):
    list = reader.groups['POINT'].get_string(name)
    elementSize = reader.groups['POINT'].get(name).dimensions[0]
    newlist = [list[i:i + elementSize] for i in range(0, len(list), elementSize)]
    for i in range(len(newlist)):
        newlist[i] = newlist[i].strip()
    return newlist

supervisor = Supervisor()
timestep = int(supervisor.getBasicTimeStep())

if len(sys.argv) < 3:
    sys.exit('C3D file not defined.')

if not os.path.isfile(sys.argv[1]):
    sys.exit('\'%s\' does not exist.' % sys.argv[1])

playbackSpeed = float(sys.argv[2])

reader = c3d.Reader(open(sys.argv[1], 'rb'))
labels = getPointsList(reader, 'LABELS')
angleLabels = getPointsList(reader, 'ANGLES')
forcesLabels = getPointsList(reader, 'FORCES')
momentsLabels = getPointsList(reader, 'MOMENTS')
powersLabels = getPointsList(reader, 'POWERS')

filteredLabel = [x for x in labels if x not in angleLabels]
filteredLabel = [x for x in filteredLabel if x not in forcesLabels]
filteredLabel = [x for x in filteredLabel if x not in momentsLabels]
filteredLabel = [x for x in filteredLabel if x not in powersLabels]

supervisor.wwiSendText(" ".join(filteredLabel))

numberOfpoints = reader.header.point_count
frameStep = 1.0 / reader.header.frame_rate
scale = reader.header.scale_factor
if reader.groups['POINT'].get('UNITS').string_value == 'mm':
    scale *= 0.001
elif not reader.groups['POINT'].get('UNITS').string_value == 'm':
    print("Can't determine the size unit.")

supervisor.step(timestep)
markerField = supervisor.getSelf().getField('markers')
pointRepresentations = {}
j = 0
for i in range(markerField.getCount()):
    markerField.removeMF(-1)

for i in range(len(labels)):
    pointRepresentations[labels[i]] = {}
    pointRepresentations[labels[i]]['visible'] = False
    pointRepresentations[labels[i]]['node'] = None
    if labels[i] in filteredLabel:
        pointRepresentations[labels[i]]['visible'] = True
        markerField.importMFNodeFromString(-1, 'C3dMarker { name "%s" }' % labels[i])
        pointRepresentations[labels[i]]['node'] = markerField.getMFNode(-1)
        pointRepresentations[labels[i]]['translation'] = pointRepresentations[labels[i]]['node'].getField('translation')
        pointRepresentations[labels[i]]['transparency'] = pointRepresentations[labels[i]]['node'].getField('transparency')
        pointRepresentations[labels[i]]['radius'] = pointRepresentations[labels[i]]['node'].getField('radius')
        pointRepresentations[labels[i]]['color'] = pointRepresentations[labels[i]]['node'].getField('color')
        j += 1

frameAndPoints = []
for i, points, analog in reader.read_frames():
    frameAndPoints.append((i, points))

frameCoutner = 0
totalFrameCoutner = 0
while supervisor.step(timestep) != -1:
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
