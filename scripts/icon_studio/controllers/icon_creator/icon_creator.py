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

"""Setup the studio, and generate continuously icons in the target directory."""

import colorsys
import json
import math
import optparse
import fnmatch
import os
import shutil
import sys

from controller import Supervisor
from PIL import Image, ImageChops

# Considerations:
# - The shadow should appear on the objects right.
# - The robot and the shadow should appear entirely in the resulted screenshot.
# - If possible, physics should have run, the object should have a dynamic pose, and the robot "eyes" should look at the camera.

RED = 0
GREEN = 1
BLUE = 2
HUE = 0
LIGHTNESS = 1
SATURATION = 2

WHITE = [1, 1, 1]

WEBOTS_HOME = os.path.normpath(os.environ['WEBOTS_HOME'])


def get_options():
    """Parse the controler arguments."""
    optParser = optparse.OptionParser()
    optParser.add_option("--disable-icon-copy", dest="disableIconCopy", action="store_true", default=False,
                         help="Disable the copy of the icons.")
    optParser.add_option("--json-file", dest="file", default="objects.json", help="Specify the JSON file to use.")
    optParser.add_option("--single-shot", dest="singleShot", action="store_true", default=False,
                         help="Take only a screenshot of the current world.")
    optParser.add_option("--appearance", dest="appearance", action="store_true", default=False,
                         help="Create the screenshot for all the appearances.")
    options, args = optParser.parse_args()
    return options


def take_original_screenshot(camera, directory):
    """Take a screenshot without alpha and crop."""
    image = camera.getImage()
    pilImage = Image.frombytes('RGBA', (camera.getWidth(), camera.getHeight()), image, 'raw', 'BGRA')
    pilImage.save(os.path.join(directory, 'original.png'))


def autocrop(im):
    """Autocrop an image based on its upperleft pixel."""
    # reference: https://stackoverflow.com/a/48605963/2210777
    rgbImage = im.convert('RGB')  # from RGBA, needed since Pillow 7.1.0
    bg = Image.new(rgbImage.mode, rgbImage.size, rgbImage.getpixel((0, 0)))
    diff = ImageChops.difference(rgbImage, bg)
    diff = ImageChops.add(diff, diff, 2.0)
    bbox = diff.getbbox()
    if bbox:
        return im.crop(bbox)
    assert False, "Impossible to crop image"


def take_screenshot(camera, category, directory, protoDirectory, protoName, options, background, colorThreshold,
                    shadowColor=None, namePostfix=''):
    """Take the screenshot."""
    # Convert Camera image to PIL image.
    image = camera.getImage()
    pilImage = Image.frombytes('RGBA', (camera.getWidth(), camera.getHeight()), image, 'raw', 'BGRA')
    pilImage = autocrop(pilImage)  # cropped at an early stage to save lot of CPU resources.
    pixels = pilImage.getdata()

    # Remove the background.
    background = [float(pixels[0][0]) / 255.0, float(pixels[0][1]) / 255.0, float(pixels[0][2]) / 255.0]
    newPixels = []
    hls_background_color = colorsys.rgb_to_hls(background[RED], background[GREEN], background[BLUE])
    for pixel in pixels:
        hls_pixel = colorsys.rgb_to_hls(float(pixel[RED]) / 255.0, float(pixel[GREEN]) / 255.0, float(pixel[BLUE]) / 255.0)
        if (abs(hls_pixel[HUE] - hls_background_color[HUE]) < colorThreshold and
                abs(hls_pixel[LIGHTNESS] - hls_background_color[LIGHTNESS]) < colorThreshold and
                abs(hls_pixel[SATURATION] - hls_background_color[SATURATION]) < colorThreshold):
            # Background
            newPixels.append((0, 0, 0, 0))
        elif (shadowColor is not None and
                shadowColor[RED] == pixel[RED] and
                shadowColor[GREEN] == pixel[GREEN] and
                shadowColor[BLUE] == pixel[BLUE]):
            # Shadows
            newPixels.append((125, 125, 125, 120))
        else:
            # Object
            newPixels.append(pixel)
    pilImage.putdata(newPixels)

    # Uncomment to show the result image:
    # pilImage.show()

    # Save model.png (cropped) and icon.png (scaled down)
    pilImage.save(os.path.join(directory, 'model.png'))

    pilImage.thumbnail((128, 128), Image.Resampling.LANCZOS)
    iconImage = Image.new('RGBA', (128, 128))
    iconImage.paste(pilImage, (int((128 - pilImage.size[0]) / 2), int((128 - pilImage.size[1]) / 2),
                    int((128 - pilImage.size[0]) / 2) + pilImage.size[0], int((128 - pilImage.size[1]) / 2) + pilImage.size[1]))
    iconImage.save(os.path.join(directory, 'icon.png'))

    if not options.disableIconCopy:
        # copy icons in the appropriate directory
        iconsFolder = os.path.join(WEBOTS_HOME, protoDirectory, 'icons')
        iconPath = os.path.join(iconsFolder, protoName + '.png')
        if not os.path.exists(iconsFolder):
            os.makedirs(iconsFolder)
        if os.path.exists(iconPath):
            os.remove(iconPath)
        shutil.copy2(os.path.join(directory, 'icon.png'), iconPath)

        categoryFolder = os.path.basename(os.path.dirname(protoDirectory))
        # copy the models in the docs directory
        modelFolder = os.path.join(WEBOTS_HOME, 'docs', 'guide', 'images', category, categoryFolder, protoName)
        modelPath = os.path.join(modelFolder, 'model' + namePostfix + '.png')
        if category == categoryFolder:  # appearances
            modelFolder = os.path.join(WEBOTS_HOME, 'docs', 'guide', 'images', category)
            modelPath = os.path.join(modelFolder, protoName + namePostfix + '.png')
        elif category == 'robots':
            modelFolder = os.path.join(WEBOTS_HOME, 'docs', 'guide', 'images', category, categoryFolder)
            modelPath = os.path.join(modelFolder, protoName + namePostfix + '.png')
        if not os.path.exists(modelFolder):
            os.makedirs(modelFolder)
        if os.path.exists(modelPath):
            os.remove(modelPath)
        shutil.copy2(os.path.join(directory, 'model.png'), modelPath)


def process_appearances(supervisor, parameters):
    """Import the appearances, take a screenshot and remove it."""
    objectDirectory = os.path.join('images', 'appearances', protoName)
    if not os.path.exists(objectDirectory):
        os.makedirs(objectDirectory)
    else:
        sys.exit('Multiple definition of ' + protoName)
    protoPath = os.path.join(rootPath, protoName)
    protoPath = protoPath.replace(WEBOTS_HOME + os.sep, '')
    nodeString = 'Transform { translation 0 0 1 rotation -1 0 0 0.262 children [ '
    nodeString += 'Shape { '
    nodeString += 'geometry Sphere { subdivision 5 } '
    nodeString += 'castShadows FALSE '
    nodeString += 'appearance %s { ' % protoName
    if 'fields' in parameters:
        assert type(parameters['fields']) is list
        postfix = 'a'
        for fields in parameters['fields']:
            newNodeString = nodeString + fields
            newNodeString += ' } } ] }'
            process_object(controller, 'appearances', newNodeString, objectDirectory,
                           protoPath, background=[1, 1, 1], colorThreshold=0.01,
                           postfix=('_' + postfix if len(parameters['fields']) > 1 else ''))
            postfix = chr(ord(postfix) + 1)
    else:
        nodeString += ' } } ] }'
        process_object(controller, 'appearances', nodeString, objectDirectory,
                       protoPath, background=[1, 1, 1], colorThreshold=0.01)


def process_object(supervisor, category, nodeString, objectDirectory, protoPath, background, colorThreshold, postfix=''):
    """Import object, take screenshot and remove it."""
    rootChildrenfield = controller.getRoot().getField('children')

    # Apply the background color.
    supervisor.getFromDef('FLOOR_MATERIAL').getField('diffuseColor').setSFColor(WHITE)

    # import the object
    count = rootChildrenfield.getCount()
    rootChildrenfield.importMFNodeFromString(-1, nodeString)
    supervisor.step(timeStep)
    if rootChildrenfield.getCount() != count + 1:
        sys.exit(protoName + ' was not imported sucessfully.')
    importedNode = rootChildrenfield.getMFNode(-1)
    supervisor.step(timeStep)

    importedNode.moveViewpoint()
    supervisor.simulationSetMode(Supervisor.SIMULATION_MODE_REAL_TIME)
    supervisor.step(60 * timeStep)

    # Set the camera at the right location.
    position = viewpointPosition.getSFVec3f()
    supervisorTranslation.setSFVec3f(position)
    supervisorRotation.setSFRotation(viewpointOrientation.getSFRotation())
    # compute distance to the object (assuming object is at the origin) to set a correct near value
    distance = math.sqrt(math.pow(position[1], 2) + math.pow(position[1], 2) + math.pow(position[1], 2))
    if distance < 1:
        cameraNear.setSFFloat(0.1)
    elif distance < 5:
        cameraNear.setSFFloat(0.2)
    elif distance < 10:
        cameraNear.setSFFloat(0.5)
    else:
        cameraNear.setSFFloat(1.0)
    supervisor.step(timeStep)

    take_original_screenshot(camera, objectDirectory)

    supervisor.getFromDef('FLOOR_MATERIAL').getField('diffuseColor').setSFColor(background)
    lightIntensityField = supervisor.getFromDef('LIGHT').getField('intensity')
    lightIntensity = lightIntensityField.getSFFloat()
    lightIntensityField.setSFFloat(0.0)
    supervisor.step(10 * timeStep)
    pixel = camera.getImageArray()[0][0]
    shadowColor = [pixel[0], pixel[1], pixel[2]]
    lightIntensityField.setSFFloat(lightIntensity)
    supervisor.step(10 * timeStep)
    take_screenshot(camera, category, objectDirectory, os.path.dirname(protoPath), protoName, options, background,
                    colorThreshold, shadowColor, postfix)

    # remove the object
    supervisor.step(timeStep)
    count = rootChildrenfield.getCount()
    importedNode.remove()
    supervisor.step(timeStep)
    if rootChildrenfield.getCount() != count - 1:
        sys.exit(protoName + ' was not removed sucessfully.')


# Initialize the Supervisor.
controller = Supervisor()
timeStep = int(controller.getBasicTimeStep())
camera = controller.getDevice('camera')
camera.enable(timeStep)
options = get_options()

if os.path.exists('images'):
    shutil.rmtree('images')

# Get required fields
rootChildrenfield = controller.getRoot().getField('children')
supervisorTranslation = controller.getFromDef('SUPERVISOR').getField('translation')
supervisorRotation = controller.getFromDef('SUPERVISOR').getField('rotation')
viewpointPosition = controller.getFromDef('VIEWPOINT').getField('position')
viewpointOrientation = controller.getFromDef('VIEWPOINT').getField('orientation')
cameraNear = controller.getFromDef('CAMERA').getField('near')

if options.singleShot:
    node = controller.getFromDef('OBJECTS')
    if node is None:
        sys.exit('No node "OBJECTS" found.')
    take_original_screenshot(camera, 'images')
    take_screenshot(camera, 'objects', 'images', os.path.dirname(controller.getWorldPath()), node.getTypeName(), None)
elif options.appearance:
    with open('appearances.json') as json_data:
        data = json.load(json_data)
        for rootPath, dirNames, fileNames in os.walk(os.path.join(WEBOTS_HOME, 'projects', 'appearances', 'protos')):
            for fileName in fnmatch.filter(fileNames, '*.proto'):
                protoName = fileName.split('.')[0]
                if protoName not in data:
                    print('Skipping "%s" PROTO.' % protoName)
                    continue
                process_appearances(controller, data[protoName])
else:
    with open(options.file) as json_data:
        data = json.load(json_data)
        print('%d objects' % (len(data) - 1))
        itemCounter = 0
        for key, value in data.items():
            if key == 'default':
                continue

            itemCounter += 1
            protoName = os.path.basename(key).split('.')[0]

            if sys.version_info[0] < 3:
                protoName = protoName.encode('utf-8')

            protoPath = key
            print('%s [%d%%]' % (protoName, 100.0 * itemCounter / (len(data) - 1)))

            objectDirectory = os.path.join('images', os.path.basename(os.path.dirname(os.path.dirname(key))), protoName)
            if not os.path.exists(objectDirectory):
                os.makedirs(objectDirectory)
            else:
                sys.exit('Multiple definition of ' + protoName)

            if 'colorThreshold' in value:
                colorThreshold = value['colorThreshold']
            else:
                colorThreshold = data['default']['colorThreshold']
            if 'background' in value:
                background = value['background']
            else:
                background = data['default']['background']
            if 'fields' in value:
                fields = value['fields']
            else:
                fields = data['default']['fields']

            nodeString = protoName + '{ '
            if sys.version_info[0] < 3:
                nodeString += fields.encode('utf-8')
            else:
                nodeString += fields
            nodeString += ' }'
            if 'nodeString' in value:
                if sys.version_info[0] < 3:
                    nodeString = value['nodeString'].encode('utf-8')
                else:
                    nodeString = value['nodeString']
            process_object(controller, key.split('/')[1], nodeString, objectDirectory, protoPath,
                           background=background, colorThreshold=colorThreshold)
