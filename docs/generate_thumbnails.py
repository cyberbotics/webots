#!/usr/bin/env python
# -*- coding: utf-8 -*-

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

import re
import os

from PIL import Image


def search_and_replace(filename, fromString, toString):
    """Search and replace string in a file."""
    with open(filename, 'r') as file:
        data = file.read()
    data = data.replace(fromString, toString)
    with open(filename, 'w') as file:
        file.write(data)


paths = [
    os.path.join('guide', 'images'),
    os.path.join('reference', 'images'),
    os.path.join('automobile', 'images')
]

# Get all the images
images = []
for path in paths:
    for root, dirnames, filenames in os.walk(path):
        for filename in filenames:
            if filename.endswith(('.jpg', '.png')) and 'thumbnail' not in filename:
                images.append(os.path.join(root, filename))

# Get all the MD files
mdFiles = []
for root, dirnames, filenames in os.walk('.'):
    for filename in filenames:
        if filename.endswith(('.md')):
            mdFiles.append(os.path.join(root, filename))

# Revert all the thumbnail in the MD files.
for mdFile in mdFiles:
    print('Revert thumbnails in "%s"', mdFile)
    search_and_replace(mdFile, '.thumbnail.png', '.png')
    search_and_replace(mdFile, '.thumbnail.jpg', '.png')

# Foreach image:
for image in images:
    im = Image.open(image)
    width, height = im.size
    print('Check image "%s" (%dx%d)' % (image, width, height))

    # Compute the expected maximum size depending on the path.
    expectedSize = 256
    if ('appearances' + os.sep) in image:
        expectedSize = 512

    # Condition to create the thumbnail.
    # - The image is big enough.
    shouldCreateThumbnail = \
        width > expectedSize or height > expectedSize

    if shouldCreateThumbnail:
        # Condition to convert to JPG.
        shouldConvertToJPG = im.mode == 'RGB'
        if im.mode == 'RGBA':
            # Take a 3 pixel samples to determine if the alpha is used.
            shouldConvertToJPG = \
                im.getpixel((0, 0))[3] == 255 and \
                im.getpixel((width / 2, height / 2))[3] == 255 and \
                im.getpixel((width - 1, height - 1))[3] == 255

        # Actually resize the image.
        im.thumbnail((expectedSize, expectedSize))
        thumbnail = image.replace('.png', '.thumbnail.png')
        targetFormat = 'PNG'

        # JPEG convertion exception.
        if shouldConvertToJPG:
            thumbnail = thumbnail.replace('.png', '.jpg')
            im = im.convert('RGB')
            targetFormat = 'JPEG'

        # Save the result.
        im.save(thumbnail, targetFormat)
        print('=> Thumbnail "%s" created (%dx%d)' % (thumbnail, im.size[0], im.size[1]))

        # Compute paths by removing books.
        imagePath = image[(image.find('/') + 1):]
        thumbnailPath = thumbnail[(thumbnail.find('/') + 1):]

        # Modify the MD files accordingly.
        for mdFile in mdFiles:
            search_and_replace(mdFile, '(%s)' % imagePath, '(%s)' % thumbnailPath)
