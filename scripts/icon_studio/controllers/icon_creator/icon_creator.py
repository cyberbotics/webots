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

WHITE = [1, 1, 1]


def get_options():
    """Parse the controler arguments."""
    optParser = optparse.OptionParser()
    optParser.add_option("--disable-icon-copy", dest="disableIconCopy", action="store_true", default=False, help="Disable the copy of the icons.")
    optParser.add_option("--json-file", dest="file", default="objects.json", help="Specify the JSON file to use.")
    optParser.add_option("--single-shot", dest="singleShot", action="store_true", default=False, help="Take only a screenshot of the current world.")
    optParser.add_option("--appearance", dest="appearance", action="store_true", default=False, help="Create the screenshot for all the appearances.")
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
    bg = Image.new(im.mode, im.size, im.getpixel((0, 0)))
    diff = ImageChops.difference(im, bg)
    diff = ImageChops.add(diff, diff, 2.0)
    bbox = diff.getbbox()
    if bbox:
        return im.crop(bbox)


def take_screenshot(camera, category, directory, protoDirectory, protoName, options, background, colorThreshold, alphaRejectionThreshold):
    """Take the screenshot."""
    # Convert Camera image to PIL image.
    image = camera.getImage()
    pilImage = Image.frombytes('RGBA', (camera.getWidth(), camera.getHeight()), image, 'raw', 'BGRA')
    pilImage = autocrop(pilImage)  # cropped at an early stage to save lot of CPU resources.
    pixels = pilImage.getdata()

    # Remove the background.
    background = [float(pixels[0][0]) / 255.0, float(pixels[0][1]) / 255.0, float(pixels[0][2]) / 255.0]
    iBackground = [1.0 - background[RED], 1.0 - background[GREEN], 1.0 - background[BLUE]]
    newPixels = []
    hls_background_color = colorsys.rgb_to_hls(background[RED], background[GREEN], background[BLUE])
    for pixel in pixels:
        hls_pixel = colorsys.rgb_to_hls(float(pixel[RED]) / 255.0, float(pixel[GREEN]) / 255.0, float(pixel[BLUE]) / 255.0)
        if abs(hls_pixel[HUE] - hls_background_color[HUE]) < colorThreshold:  # If pixel color is close to background.
            colorChanel = int((iBackground[RED] * pixel[RED] + iBackground[GREEN] * pixel[GREEN] + iBackground[BLUE] * pixel[BLUE]) / (iBackground[RED] + iBackground[GREEN] + iBackground[BLUE]))
            alphaChanel = int(255 - (background[RED] * pixel[RED] + background[GREEN] * pixel[GREEN] + background[BLUE] * pixel[BLUE]) / (background[RED] + background[GREEN] + background[BLUE]))
            if alphaChanel < alphaRejectionThreshold * 255:
                alphaChanel = 0
            newPixels.append((colorChanel, colorChanel, colorChanel, alphaChanel))
        else:
            newPixels.append(pixel)
    pilImage.putdata(newPixels)

    # Uncomment to show the result image:
    # pilImage.show()

    # Save model.png (cropped) and icon.png (scaled down)
    pilImage.save(os.path.join(directory, 'model.png'))

    pilImage.thumbnail((128, 128), Image.ANTIALIAS)
    iconImage = Image.new('RGBA', (128, 128))
    iconImage.paste(pilImage, (int((128 - pilImage.size[0]) / 2), int((128 - pilImage.size[1]) / 2), int((128 - pilImage.size[0]) / 2) + pilImage.size[0], int((128 - pilImage.size[1]) / 2) + pilImage.size[1]))
    iconImage.save(os.path.join(directory, 'icon.png'))

    if not options.disableIconCopy:
        # copy icons in the appropriate directory
        iconsFolder = os.environ['WEBOTS_HOME'] + os.sep + protoDirectory + os.sep + 'icons'
        iconPath = iconsFolder + os.sep + protoName + '.png'
        if not os.path.exists(iconsFolder):
            os.makedirs(iconsFolder)
        if os.path.exists(iconPath):
            os.remove(iconPath)
        shutil.copy2(directory + os.sep + 'icon.png', iconPath)

        categoryFolder = os.path.basename(os.path.dirname(protoDirectory))
        # copy the models in the docs directory
        modelFolder = os.path.join(os.environ['WEBOTS_HOME'], 'docs', 'guide', 'images', category, categoryFolder, protoName)
        modelPath = os.path.join(modelFolder, 'model.png')
        if category == categoryFolder:
            modelFolder = os.path.join(os.environ['WEBOTS_HOME'], 'docs', 'guide', 'images', category)
            modelPath = os.path.join(modelFolder, protoName + '.png')
        elif category == 'robots':
            modelFolder = os.path.join(os.environ['WEBOTS_HOME'], 'docs', 'guide', 'images', category, categoryFolder)
            modelPath = os.path.join(modelFolder, protoName + '.png')
        if not os.path.exists(modelFolder):
            os.makedirs(modelFolder)
        if os.path.exists(modelPath):
            os.remove(modelPath)
        shutil.copy2(directory + os.sep + 'model.png', modelPath)


def process_object(supervisor, category, nodeString, background, colorThreshold, alphaRejectionThreshold):
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
    distance = math.sqrt(math.pow(position[0], 2) + math.pow(position[0], 2) + math.pow(position[0], 2))
    if distance < 1:
        cameraNear.setSFFloat(0.1)
    elif distance < 5:
        cameraNear.setSFFloat(0.2)
    elif distance < 10:
        cameraNear.setSFFloat(0.5)
    else:
        cameraNear.setSFFloat(1)
    supervisor.step(timeStep)

    take_original_screenshot(camera, objectDirectory)

    supervisor.getFromDef('FLOOR_MATERIAL').getField('diffuseColor').setSFColor(background)
    supervisor.step(10 * timeStep)
    take_screenshot(camera, category, objectDirectory, os.path.dirname(protoPath), protoName, options, background, colorThreshold, alphaRejectionThreshold)

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
camera = controller.getCamera('camera')
camera.enable(timeStep)
options = get_options()

if os.path.exists('.' + os.sep + 'images'):
    shutil.rmtree('.' + os.sep + 'images')

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
    take_original_screenshot(camera, '.' + os.sep + 'images')
    take_screenshot(camera, 'objects', '.' + os.sep + 'images', os.path.dirname(controller.getWorldPath()), node.getTypeName(), None)
elif options.appearance:
    with open('appearances.json') as json_data:
        data = json.load(json_data)
        appearanceFolder = os.path.join(os.environ['WEBOTS_HOME'], 'projects')
        appearanceFolder = os.path.join(appearanceFolder, 'appearances')
        appearanceFolder = os.path.join(appearanceFolder, 'protos')
        for rootPath, dirNames, fileNames in os.walk(appearanceFolder):
                for fileName in fnmatch.filter(fileNames, '*.proto'):
                    protoName = fileName.split('.')[0]
                    protoPath = rootPath + os.sep + protoName
                    protoPath = protoPath.replace(os.environ['WEBOTS_HOME'], '')
                    nodeString = 'Transform { translation 0 1 0 rotation 0 0 1 0.262 children [ '
                    nodeString += 'Shape { appearance %s { ' % protoName
                    if protoName in data:
                        parameters = data[protoName]
                        if 'fields' in parameters:
                            nodeString += parameters['fields']
                    nodeString += ' } '
                    nodeString += 'geometry Sphere { subdivision 6 } } ] }'

                    objectDirectory = '.' + os.sep + 'images' + os.sep + 'appearances' + os.sep + protoName
                    if not os.path.exists(objectDirectory):
                        os.makedirs(objectDirectory)
                    else:
                        sys.exit('Multiple definition of ' + protoName)
                    process_object(controller, 'appearances', nodeString, background=[0, 1, 0], colorThreshold=0.1, alphaRejectionThreshold=0.6)
else:
    with open(options.file) as json_data:
        data = json.load(json_data)
        print('%d objects' % (len(data) - 1))
        itemCounter = 0
        for key, value in data.items():
            if key == 'default':
                continue

            itemCounter += 1
            protoName = os.path.basename(key).split('.')[0].encode('utf-8')
            protoPath = key
            print('%s [%d%%]' % (protoName, 100.0 * itemCounter / (len(data) - 1)))

            objectDirectory = '.' + os.sep + 'images' + os.sep + os.path.basename(os.path.dirname(os.path.dirname(key))) + os.sep + protoName
            if not os.path.exists(objectDirectory):
                os.makedirs(objectDirectory)
            else:
                sys.exit('Multiple definition of ' + protoName)

            if 'colorThreshold' in value:
                colorThreshold = value['colorThreshold']
            else:
                colorThreshold = data['default']['colorThreshold']
            if 'alphaRejectionThreshold' in value:
                alphaRejectionThreshold = value['alphaRejectionThreshold']
            else:
                alphaRejectionThreshold = data['default']['alphaRejectionThreshold']
            if 'background' in value:
                background = value['background']
            else:
                background = data['default']['background']
            if 'fields' in value:
                fields = value['fields']
            else:
                fields = data['default']['fields']

            nodeString = protoName + '{ '
            nodeString += fields.encode('utf-8')
            nodeString += ' }'
            if 'nodeString' in value:
                nodeString = value['nodeString'].encode('utf-8')

            process_object(controller, key.split('/')[1], nodeString, background=[0, 1, 1], colorThreshold=0.05, alphaRejectionThreshold=0.4)
