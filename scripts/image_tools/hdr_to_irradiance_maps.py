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

"""Convert an HDR image to HDR irradiance maps."""
# The irradiance maps are generated in the same directory as the input HDR image.

import math
import optparse
import os
import sys

# from clamp import clamp_int
from hdr import HDR
from vec3 import Vec3

optParser = optparse.OptionParser(usage='usage: %prog --input=image.hdr')
optParser.add_option(
    '--input', '-i', dest='input', default='image.hdr', type='string',
    help='specifies the input HDR image path'
)
options, args = optParser.parse_args()

hdr_path = options.input
irradiance_map_path = hdr_path.replace('.hdr', '.dm.hdr')

assert hdr_path.endswith('.hdr'), 'Invalid input extension.'
assert hdr_path != irradiance_map_path, 'Identical input and output paths.'
assert os.path.isfile(hdr_path), 'Input file doest not exits.'

print('Load the HDR image...')
hdr = HDR.load_from_file(hdr_path)
assert hdr.is_valid(), 'Invalid input HDR file.'


def drange(start, stop, step):
    r = start
    while r < stop:
        yield r
        r += step


print('Create the diffuse irradiance map...')
size = 32
irradiance_map = HDR.create_black_image(size, size)
for y in range(size):
    sys.stdout.write("\r %3.0f %%" % (100.0 * (1.0 + y) / size))
    sys.stdout.flush()
    y0 = float(y) / size - 0.5
    for x in range(size):
        x0 = float(x) / size - 0.5
        N = Vec3(x0, y0, -1.0).normalize()
        irradiance = Vec3(0.0, 0.0, 0.0)
        up = Vec3(0.0, 1.0, 0.0)
        right = up.cross(N)
        up = N.cross(right)

        sampleDelta = 0.025
        nrSamples = 0.0
        for phi in drange(0.0, 2.0 * math.pi, sampleDelta):
            for theta in drange(0.0, 0.5 * math.pi, sampleDelta):
                tangentSample = Vec3(math.sin(theta) * math.cos(phi), math.sin(theta) * math.sin(phi), math.cos(theta))
                sampleVec = right * tangentSample.x + up * tangentSample.y + N * tangentSample.z

                p1 = hdr.get_pixel(int(hdr.width * (sampleVec.x / 2.0 + 0.5)), int(hdr.height * (sampleVec.y / 2.0 + 0.5)))  # A picking in the entire HDR cubemap is required.
                p1 = Vec3(p1[0], p1[1], p1[2])
                irradiance += p1 * math.cos(theta) * math.sin(theta)
                nrSamples += 1.0
        irradiance = irradiance * math.pi / nrSamples
        pixel = (irradiance.x, irradiance.y, irradiance.z)
        irradiance_map.set_pixel(x, y, pixel)
sys.stdout.write("\n")
irradiance_map.save(irradiance_map_path)
