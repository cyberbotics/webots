#!/usr/bin/env python

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

"""Generate textures prepared for OSM, based on image templates."""

import glob
import os
from PIL import Image


# change directory to this script directory in order to allow this script to be called from another directory.
os.chdir(os.path.dirname(os.path.realpath(__file__)))

# get all the template files in put them in a list of tuples
templates = []
for f in glob.glob("*_diffuse_template.jpg"):
    templates.append((f, f.replace('_diffuse_', '_color_mask_')))

# target colors
# ref: http://wiki.openstreetmap.org/wiki/Key:colour
# TODO: is it sufficient?
colors = {
    '000000': (0.0, 0.0, 0.0),
    'FFFFFF': (0.84, 0.84, 0.84),
    '808080': (0.4, 0.4, 0.4),
    'C0C0C0': (0.65, 0.65, 0.65),
    '800000': (0.4, 0.15, 0.15),
    'FF0000': (0.45, 0.0, 0.0),
    '808000': (0.4, 0.4, 0.2),
    'FFFF00': (0.7, 0.6, 0.15),
    '008000': (0.15, 0.3, 0.15),
    '00FF00': (0.55, 0.69, 0.52),
    '008080': (0.15, 0.3, 0.3),
    '00FFFF': (0.6, 0.7, 0.7),
    '000080': (0.2, 0.2, 0.3),
    '0000FF': (0.4, 0.4, 0.75),
    '800080': (0.5, 0.4, 0.5),
    'FF00FF': (0.9, 0.75, 0.85),
    'F5DEB3': (0.83, 0.78, 0.65),
    '8B4513': (0.3, 0.1, 0.05)
}

effectFactor = 0.5  # power of the effect, found empirically

# foreach template
for template in templates:
    # load the templates
    diffuse = Image.open(template[0])
    mask = Image.open(template[1])
    assert diffuse.size == mask.size
    width, height = diffuse.size

    # create an image per color
    for colorString, color in colors.iteritems():
        image = Image.new('RGB', diffuse.size)
        pixels = image.load()
        for x in range(height):
            for y in range(width):
                dR, dG, dB = diffuse.getpixel((x, y))
                mR, mG, mB = mask.getpixel((x, y))
                r = dR + int(255.0 * (mR / 255.0) * (color[0] * 2.0 - 1.0) * effectFactor)
                g = dG + int(255.0 * (mG / 255.0) * (color[1] * 2.0 - 1.0) * effectFactor)
                b = dB + int(255.0 * (mB / 255.0) * (color[2] * 2.0 - 1.0) * effectFactor)
                pixels[x, y] = (r, g, b)
        image.save(template[0].replace('_diffuse_template', '_' + colorString))
