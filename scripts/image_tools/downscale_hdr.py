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

"""Downscale an HDR image to a specified size."""
# Look out: this scripts overrides the input image.

import math
import argparse
import os

from images.hdr import HDR
from utils.range import clamp_int

parser = argparse.ArgumentParser(description='Downscale an HDR image to a specified size')
parser.add_argument(
    '--input', '-i', dest='input', default='image.hdr',
    help='specifies the input HDR image path'
)
parser.add_argument(
    '--width', dest='width', default=256, type=int,
    help='specifies the width of the target image.'
)
parser.add_argument(
    '--height', dest='height', default=256, type=int,
    help='specifies the height of the target image.'
)
args = parser.parse_args()

hdr_path = args.input

assert hdr_path.endswith('.hdr'), 'Invalid input extension.'
assert os.path.isfile(hdr_path), 'Input file doest not exits.'

print('Load the HDR image...')
hdr = HDR.load_from_file(hdr_path)
assert hdr.is_valid(), 'Invalid input HDR file.'

print('Create the result image...')
result = HDR.create_black_image(args.width, args.height)
for y in range(args.height):
    for x in range(args.width):
        uf = float(x) * hdr.width / args.width
        vf = float(y) * hdr.height / args.height
        u1 = int(math.floor(uf))  # coord of pixel to bottom left
        v1 = int(math.floor(vf))
        u2 = u1 + 1  # coords of pixel to top right
        v2 = v1 + 1
        mu = uf - u1  # fraction of way across pixel
        nu = vf - v1

        # Bilinear interpolation
        A = hdr.get_pixel(u1 % hdr.width, clamp_int(v1, 0, hdr.height - 1))
        B = hdr.get_pixel(u2 % hdr.width, clamp_int(v1, 0, hdr.height - 1))
        C = hdr.get_pixel(u1 % hdr.width, clamp_int(v2, 0, hdr.height - 1))
        D = hdr.get_pixel(u2 % hdr.width, clamp_int(v2, 0, hdr.height - 1))
        P = (
            A[0] * (1 - mu) * (1 - nu) + B[0] * (mu) * (1 - nu) + C[0] * (1 - mu) * nu + D[0] * mu * nu,
            A[1] * (1 - mu) * (1 - nu) + B[1] * (mu) * (1 - nu) + C[1] * (1 - mu) * nu + D[1] * mu * nu,
            A[2] * (1 - mu) * (1 - nu) + B[2] * (mu) * (1 - nu) + C[2] * (1 - mu) * nu + D[2] * mu * nu
        )
        result.set_pixel(x, y, P)
result.save(hdr_path)
