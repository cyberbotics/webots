"""Create a robot component scene foreach robot of the robots.json file."""

# Typical command to run:
# `./webots --enable-x3d-meta-file-export --mode=fast --minimize private_projects/robot_component_studio/worlds/robot_component_studio.wbt`

from controller import Supervisor
import json
import os
import shutil
from lxml import etree

def _compareDevice(d1, d2):
    priortyDeviceTypes = ['RotationalMotor', 'LinearMotor', 'LED']  # Device types appearing first.
    for priortyDeviceType in priortyDeviceTypes:
        if d1['type'] == priortyDeviceType and d2['type'] == priortyDeviceType:
            return cmp(d1['name'].lower(), d2['name'].lower())
        elif d1['type'] == priortyDeviceType:
            return -1
        elif d2['type'] == priortyDeviceType:
            return 1
    return cmp(d1['name'].lower(), d2['name'].lower())


userGuidePath = os.path.join(os.getenv('WEBOTS_HOME'), 'docs', 'guide')

supervisor = Supervisor()
timeStep = int(supervisor.getBasicTimeStep())

robot = supervisor.getFromDef('ROBOT')
robotName = robot.getField('name').getSFString()

# Get target paths.
scenePath = os.path.join(userGuidePath, 'scenes', robotName)
targetHTMLFile = os.path.join(scenePath, robotName + '.html')
targetAnimationFile = os.path.join(scenePath, robotName + '.json')
targetMetaFile = os.path.join(scenePath, robotName + '.meta.json')
targetX3DFile = os.path.join(scenePath, robotName + '.x3d')

# Store the scene.
if os.path.exists(scenePath):
    shutil.rmtree(scenePath)
if not os.path.exists(scenePath):
    os.makedirs(scenePath)
supervisor.animationStartRecording(targetHTMLFile)
supervisor.animationStopRecording()
supervisor.step(timeStep)
supervisor.step(timeStep)

# Remove useless files.
os.remove(targetHTMLFile)
os.remove(targetAnimationFile)

# Simplified JSON file.
# - keep only the interested robot.
assert os.path.exists(targetMetaFile), 'The meta file does not exists. Please run Webots with the "--enable-x3d-meta-file-export" argument.'
robotsMetaData = json.load(open(targetMetaFile))
robotData = None
for _robotData in robotsMetaData:
    if _robotData['name'] == robotName:
        robotData = _robotData
        break
assert robotData, 'Failed to simplified the JSON supervisor.'
# - sort the device list per interesting category type.
robotData['devices'] = sorted(robotData['devices'], cmp=_compareDevice)
# - rewrite the json file.
with open(targetMetaFile, 'w') as f:
    json.dump(robotData, f, indent=2)
    f.write('\n')

# Hard-code the shadows parameters (to be independant on the export settings).
tree = etree.parse(targetX3DFile)
lights = tree.xpath('//DirectionalLight')
lights[0].attrib['shadowIntensity'] = '0.5'
lights[0].attrib['shadowMapSize'] = '512'
lights[0].attrib['shadowFilterSize'] = '2'
lights[0].attrib['shadowCascades'] = '3'
tree.write(targetX3DFile, pretty_print=True, xml_declaration=True, encoding="utf-8")

supervisor.step(timeStep)

supervisor.simulationQuit(0)
