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

from clamp import clamp_int
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
irradiance_map = HDR.create_black_image(32, 32)
for y in range(32):
    for x in range(32):
        N = Vec3(0.0, 0.0, 0.0)  # hu?
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

                p1 = hdr.get_pixel(int(sampleVec.x), int(sampleVec.y))
                p1 = Vec3(p1[0], p1[1], p1[2])
                irradiance += p1 * math.cos(theta) * math.sin(theta)
                nrSamples += 1.0
        irradiance = irradiance * math.pi * (1.0 / float(nrSamples))

        pixel = (irradiance.x, irradiance.y, irradiance.z)
        irradiance_map.set_pixel(x, y, pixel)
irradiance_map.save(irradiance_map_path)
