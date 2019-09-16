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

from clamp import clamp_int
from hdr import HDR
from vec3 import Vec3

optParser = optparse.OptionParser(usage='usage: %prog --input=image.hdr')
optParser.add_option(
    '--input-dir', '-d', dest='dir', default='.', type='string',
    help='specifies the path containing the input HDR images'
)
optParser.add_option(
    '--input-name', '-n', dest='name', default='cubemap', type='string',
    help='specifies the common name prefix of the HDR images'
)
options, args = optParser.parse_args()

suffixes = ['right', 'left', 'top', 'bottom', 'front', 'back']

hdr_paths = []
diffuse_irradiance_map_paths = []
for suffix in suffixes:
    hdr_paths.append(os.path.join(options.dir, options.name + '_' + suffix + '.hdr'))
    diffuse_irradiance_map_paths.append(os.path.join(options.dir, options.name + '_' + suffix + '.dm.hdr'))

for i in range(len(hdr_paths)):
    assert hdr_paths[i] != diffuse_irradiance_map_paths[i], 'Identical input and output paths.'
    assert os.path.isfile(hdr_paths[i]), 'Input file doest not exits.'

hdrs = []
for path in hdr_paths:
    print('Load the "%s"...' % path)
    hdr = HDR.load_from_file(path)
    assert hdr.is_valid(), 'Invalid input HDR file.'
    hdrs.append(hdr)


def drange(start, stop, step):
    r = start
    while r < stop:
        yield r
        r += step


def cubemap_lookup(direction):
    abs = direction.abs()
    ma = 0.0
    u = 0.0
    v = 0.0
    fi = 0
    # cubemap directions:
    # right = 1.0f, 0.0f, 0.0f
    # left = -1.0f, 0.0f, 0.0f
    # top = 0.0f, 1.0f, 0.0f
    # bottom = 0.0f, -1.0f, 0.0f
    # front = 0.0f, 0.0f, 1.0f
    # back = 0.0f, 0.0f, -1.0f
    if abs.z >= abs.x and abs.z >= abs.y:
        fi = 5 if direction.z < 0.0 else 4
        ma = 0.5 / abs.z
        u = -direction.x if direction.z < 0.0 else direction.x
        v = -direction.y
    elif abs.y >= abs.x:
        fi = 3 if direction.y < 0.0 else 2
        ma = 0.5 / abs.y
        u = direction.x
        v = -direction.z if direction.y < 0.0 else direction.z
    else:
        fi = 1 if direction.x < 0.0 else 0
        ma = 0.5 / abs.x
        u = direction.z if direction.x < 0.0 else -direction.z
        v = -direction.y
    return (u * ma + 0.5, v * ma + 0.5, fi)


for i in range(len(diffuse_irradiance_map_paths)):
    print('Create the "%s" irradiance map...' % diffuse_irradiance_map_paths[i])
    size = 32
    irradiance_map = HDR.create_black_image(size, size)
    for y in range(size):
        sys.stdout.write('\r %3.0f %%' % (100.0 * (1.0 + y) / size))
        sys.stdout.flush()
        y0 = 2.0 * (float(y) / size) - 1.0
        N = Vec3()
        for x in range(size):
            x0 = 2.0 * (float(x) / size) - 1.0
            if i == 0:
                N = Vec3(1.0, y0, -x0).normalize()
            elif i == 1:
                N = Vec3(-1.0, y0, -x0).normalize()
            elif i == 2:
                N = Vec3(x0, 1.0, y0).normalize()
            elif i == 3:
                N = Vec3(x0, -1.0, y0).normalize()
            elif i == 4:
                N = Vec3(x0, -y0, 1.0).normalize()
            elif i == 5:
                N = Vec3(x0, -y0, -1.0).normalize()
            irradiance = Vec3(0.0, 0.0, 0.0)
            up = Vec3(0.0, 1.0, 0.0)
            right = up.cross(N)
            up = N.cross(right)

            sampleDelta = 0.025
            nrSamples = 0
            for phi in drange(0.0, 2.0 * math.pi, sampleDelta):
                for theta in drange(0.0, 0.5 * math.pi, sampleDelta):
                    tangentSample = Vec3(math.sin(theta) * math.cos(phi), math.sin(theta) * math.sin(phi), math.cos(theta))
                    sampleVec = right * tangentSample.x + up * tangentSample.y + N * tangentSample.z
                    (u, v, fi) = cubemap_lookup(sampleVec)
                    p1 = hdrs[i].get_pixel(
                        clamp_int(hdrs[fi].width * u, 0, hdrs[fi].width - 1),
                        clamp_int(hdrs[fi].height * v, 0, hdrs[fi].height - 1)
                    )
                    p1 = Vec3(p1[0], p1[1], p1[2])
                    irradiance += p1 * math.cos(theta) * math.sin(theta)
                    nrSamples += 1
            irradiance = irradiance * math.pi / nrSamples
            pixel = (irradiance.x, irradiance.y, irradiance.z)
            irradiance_map.set_pixel(x, y, pixel)
    sys.stdout.write('\n')
    irradiance_map.save(diffuse_irradiance_map_paths[i])
