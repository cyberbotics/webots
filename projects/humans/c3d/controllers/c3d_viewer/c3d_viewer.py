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

if len(sys.argv) < 2:
    sys.exit('C3D file not defined.')

if not os.path.isfile(sys.argv[1]):
    sys.exit('\'%s\' does not exist.' % sys.argv[1])

reader = c3d.Reader(open(sys.argv[1], 'rb'))
print('Header:')
print(reader.header)
labels = getPointsList(reader, 'LABELS')
angleLabels = getPointsList(reader, 'ANGLES')
forcesLabels = getPointsList(reader, 'FORCES')
momentsLabels = getPointsList(reader, 'MOMENTS')
powersLabels = getPointsList(reader, 'POWERS')

filteredLabel = [x for x in labels if x not in angleLabels]
filteredLabel = [x for x in filteredLabel if x not in forcesLabels]
filteredLabel = [x for x in filteredLabel if x not in momentsLabels]
filteredLabel = [x for x in filteredLabel if x not in powersLabels]

print(filteredLabel)

supervisor.wwiSendText(" ".join(filteredLabel))

numberOfpoints = reader.header.point_count

scale = reader.header.scale_factor
if reader.groups['POINT'].get('UNITS').string_value == 'mm':
    scale *= 0.001
else:
    print("Can't determine the size unit.")

markerField = supervisor.getSelf().getField('markers')
pointRepresentations = {}
j = 0
for i in range(len(labels)):
    pointRepresentations[labels[i]] = {}
    pointRepresentations[labels[i]]['visible'] = False
    pointRepresentations[labels[i]]['node'] = None
    if labels[i] in filteredLabel:
        pointRepresentations[labels[i]]['visible'] = True
        markerField.importMFNodeFromString(-1, 'C3dMarker { }')
        pointRepresentations[labels[i]]['node'] = markerField.getMFNode(-1)
        pointRepresentations[labels[i]]['translation'] = pointRepresentations[labels[i]]['node'].getField('translation')
        pointRepresentations[labels[i]]['transparency'] = pointRepresentations[labels[i]]['node'].getField('transparency')
        pointRepresentations[labels[i]]['radius'] = pointRepresentations[labels[i]]['node'].getField('radius')
        pointRepresentations[labels[i]]['color'] = pointRepresentations[labels[i]]['node'].getField('color')
        j += 1

frameAndPoints = []
for i, points, analog in reader.read_frames():
    frameAndPoints.append((i, points))

i = 0
while supervisor.step(timestep) != -1:
    message = supervisor.wwiReceiveText()
    while message:
        print(message)
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
    frame = frameAndPoints[i][0]
    points = frameAndPoints[i][1]

    for j in range(numberOfpoints):
        if pointRepresentations[labels[j]]['visible']:
            x = points[j][0] * scale
            y = -points[j][2] * scale
            z = points[j][1] * scale
            pointRepresentations[labels[j]]['node'].getField('translation').setSFVec3f([x, y, z])
    i += 1
    if i >= len(frameAndPoints):
        i = 0
