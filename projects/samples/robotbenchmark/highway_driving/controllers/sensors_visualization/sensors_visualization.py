"""Simple supervisor used to change the color of the 'DistanceSensorVisualization'."""

import math

from controller import Supervisor

TIME_STEP = 50

# name of the available distance sensors
sensorsNames = [
    'front',
    'front right 0',
    'front right 1',
    'front right 2',
    'front left 0',
    'front left 1',
    'front left 2',
    'rear',
    'rear left',
    'rear right',
    'right',
    'left']

sensors = {}
colorFields = {}

supervisor = Supervisor()

# get and enable the distance sensors
for name in sensorsNames:
    sensors[name] = supervisor.getDistanceSensor('distance sensor ' + name)
    sensors[name].enable(TIME_STEP)
    defName = name.upper().replace(' ', '_')
    colorFields[name] = supervisor.getFromDef(defName + '_VISUALIZATION').getField('diffuseColor')

# get the color fields
childrenField = supervisor.getSelf().getField('children')
for i in range(childrenField.getCount()):
    node = childrenField.getMFNode(i)
    if node.getTypeName() == 'DistanceSensorVisualization':
        colorFields[node.getDef()] = node.getField('diffuseColor')
        colorFields[node.getDef()].setSFColor([0.0, 1.0, 0.0])

while supervisor.step(TIME_STEP) != -1:
    # adjust the color according to the value returned by the front distance sensor
    for name in sensorsNames:
        ratio = math.pow(sensors[name].getValue() / sensors[name].getMaxValue(), 2.0)
        colorFields[name].setSFColor([1.0 - ratio, 0.0, ratio])
