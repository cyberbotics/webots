#!/usr/bin/env python3

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

"""Convert an image having an equirectanglar projection to 6 cubemap images."""
# Support HDR, PNG and JPG formats.

# Reference about the projection:
# - https://stackoverflow.com/a/36976448

import argparse
import os
import math

from images.hdr import HDR
from images.regular_image import RegularImage
from utils.range import clamp_int

parser = argparse.ArgumentParser(description='Convert an image having an equirectanglar projection to 6 cubemap images')
parser.add_argument(
    '--input', '-i', dest='input', default='image.hdr',
    help='specifies the input equirectangle image path'
)
parser.add_argument(
    '--width', dest='width', default=1024, type=int,
    help='specifies the width of the target cubemap images'
)
parser.add_argument(
    '--height', dest='height', default=1024, type=int,
    help='specifies the height of the target cubemap images'
)
args = parser.parse_args()

cubemap_names = ['back', 'left', 'front', 'right', 'top', 'bottom']


def face_direction(i, j, faceID, faceWidth, faceHeight):
    a = 2.0 * float(i) / faceWidth
    b = 2.0 * float(j) / faceHeight

    if faceID == 0:  # back
        return (-1.0, 1.0 - a, 1.0 - b)
    elif faceID == 1:  # left
        return (a - 1.0, -1.0, 1.0 - b)
    elif faceID == 2:  # front
        return (1.0, a - 1.0, 1.0 - b)
    elif faceID == 3:  # right
        return (1.0 - a, 1.0, 1.0 - b)
    elif faceID == 4:  # top
        return (b - 1.0, a - 1.0, 1.0)
    else:  # faceID == 5:  # bottom
        return (1.0 - b, a - 1.0, -1.0)


print('Load the equirectangular image...')
basename, extension = os.path.splitext(args.input)
image_class = HDR if extension == '.hdr' else RegularImage
equi = image_class.load_from_file(args.input)

for c in range(len(cubemap_names)):
    cm_name = cubemap_names[c]
    print('Generate %s cubemap image...' % cm_name)
    cm_filename = args.input.replace(extension, '_' + cm_name + extension)
    cubemap = image_class.create_black_image(args.width, args.height)
    for i in range(args.height):
        for j in range(args.width):
            (x, y, z) = face_direction(i, j, c, args.width, args.height)

            theta = math.atan2(y, x)  # range -pi to pi
            r = math.hypot(x, y)
            phi = math.atan2(z, r)  # range -pi/2 to pi/2

            uf = -0.5 * equi.width * (math.pi / 2.0 + theta) / math.pi
            vf = equi.height * (math.pi / 2.0 - phi) / math.pi
            u1 = int(math.floor(uf))  # coord of pixel to bottom left
            v1 = int(math.floor(vf))
            u2 = u1 + 1  # coords of pixel to top right
            v2 = v1 + 1
            mu = uf - u1  # fraction of way across pixel
            nu = vf - v1

            # Bilinear interpolation
            A = equi.get_pixel(u1 % equi.width, clamp_int(v1, 0, equi.height - 1))
            B = equi.get_pixel(u2 % equi.width, clamp_int(v1, 0, equi.height - 1))
            C = equi.get_pixel(u1 % equi.width, clamp_int(v2, 0, equi.height - 1))
            D = equi.get_pixel(u2 % equi.width, clamp_int(v2, 0, equi.height - 1))
            P = (
                A[0] * (1 - mu) * (1 - nu) + B[0] * (mu) * (1 - nu) + C[0] * (1 - mu) * nu + D[0] * mu * nu,
                A[1] * (1 - mu) * (1 - nu) + B[1] * (mu) * (1 - nu) + C[1] * (1 - mu) * nu + D[1] * mu * nu,
                A[2] * (1 - mu) * (1 - nu) + B[2] * (mu) * (1 - nu) + C[2] * (1 - mu) * nu + D[2] * mu * nu
            )
            cubemap.set_pixel(i, j, P)
    cubemap.save(cm_filename)
