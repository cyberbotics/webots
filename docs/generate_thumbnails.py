#!/usr/bin/env python
# -*- coding: utf-8 -*-

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

import os
import sys

from PIL import Image


def search_and_replace(filename, fromString, toString):
    """Search and replace string in a file."""
    with open(filename, 'r') as file:
        data = file.read()
    data = data.replace(fromString, toString)
    with open(filename, 'w') as file:
        file.write(data)


fileDirectory = os.path.dirname(os.path.abspath(__file__))
paths = [
    os.path.join(fileDirectory, 'automobile', 'images'),
    os.path.join(fileDirectory, 'blog', 'images'),
    os.path.join(fileDirectory, 'discord', 'images'),
    os.path.join(fileDirectory, 'guide', 'images'),
    os.path.join(fileDirectory, 'reference', 'images')
]

# Get all the images and remove the thumbnails.
images = []
for path in paths:
    for root, dirnames, filenames in os.walk(path):
        for filename in filenames:
            if filename.endswith(('.jpg', '.png')):
                image = os.path.join(root, filename)
                if 'thumbnail' not in filename:
                    images.append(image)
                else:
                    os.remove(image)

# Get all the MD files
mdFiles = []
for root, dirnames, filenames in os.walk(fileDirectory):
    for filename in filenames:
        if filename.endswith(('.md')):
            mdFiles.append(os.path.join(root, filename))

# Revert all the thumbnail in the MD files.
for mdFile in mdFiles:
    if '--silent' not in sys.argv:
        print('Revert thumbnails in "%s"', mdFile)
    search_and_replace(mdFile, '.thumbnail.png', '.png')
    search_and_replace(mdFile, '.thumbnail.jpg', '.png')

# Foreach image:
for image in images:
    im = Image.open(image)
    width, height = im.size
    if '--silent' not in sys.argv:
        print('Check image "%s" (%dx%d)' % (image, width, height))

    # Compute the expected maximum size depending on the path.
    expectedSize = 400
    if '.wbt.' in image:
        expectedSize = 256

    # Condition to create the thumbnail.
    # - The image is big enough and it is meaningful to do it.
    shouldCreateThumbnail = \
        width > 1.5 * expectedSize or height > 1.5 * expectedSize

    if shouldCreateThumbnail:
        # Condition to convert to JPG.
        shouldConvertToJPG = im.mode == 'RGB'
        if im.mode == 'RGBA':
            # Take a 3 pixel samples to determine if the alpha is used.
            shouldConvertToJPG = \
                im.getpixel((1, 1))[3] == 255 and \
                im.getpixel((width / 2, height / 2))[3] == 255 and \
                im.getpixel((width - 2, height - 2))[3] == 255

        # Actually resize the image.
        im.thumbnail((expectedSize, expectedSize))
        thumbnail = image.replace('.png', '.thumbnail.png')
        targetFormat = 'PNG'

        # JPEG convertion exception.
        if image.endswith('.jpg') or shouldConvertToJPG:
            thumbnail = thumbnail.replace('.png', '.jpg')
            targetFormat = 'JPEG'
        if shouldConvertToJPG:
            im = im.convert('RGB')

        # Save the result.
        im.save(thumbnail, targetFormat)
        if '--silent' not in sys.argv:
            print('=> Thumbnail "%s" created (%dx%d)' % (thumbnail, im.size[0], im.size[1]))

        # Compute paths by removing books.
        image = image.replace(fileDirectory + os.sep, '')
        book = image[:image.find('/')]
        imagePath = image[(image.find('/') + 1):]
        thumbnail = thumbnail.replace(fileDirectory + os.sep, '')
        thumbnailPath = thumbnail[(thumbnail.find('/') + 1):]

        # Modify the MD files accordingly.
        for mdFile in mdFiles:
            mdFile = mdFile.replace(fileDirectory + os.sep, '')
            if mdFile.startswith(book):
                search_and_replace(os.path.join(fileDirectory, mdFile),
                                   '(%s)' % imagePath, '(%s)' % thumbnailPath)  # Usual image references.
                search_and_replace(os.path.join(fileDirectory, mdFile),
                                   ' %s' % imagePath, ' %s' % thumbnailPath)  # Web component fallbacks.
