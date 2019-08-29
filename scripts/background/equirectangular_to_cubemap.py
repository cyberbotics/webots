#!/usr/bin/python

"""Convert a equirectanglar HDR image to cubemap HDR images."""
# Reference about the projection:
# - https://stackoverflow.com/a/36976448

import optparse
import math

from hdr import HDR
from clamp import clamp_int

optParser = optparse.OptionParser(usage='usage: %prog --input=image.hdr [options]')
optParser.add_option(
    '--input', '-i', dest='input', default='image.hdr', type='string',
    help='specifies the input equirectangle image path'
)
optParser.add_option(
    '--width', dest='width', default=1024, type='int',
    help='specifies the width of the target cubemap images'
)
optParser.add_option(
    '--height', dest='height', default=1024, type='int',
    help='specifies the height of the target cubemap images'
)
options, args = optParser.parse_args()

cubemap_names = [
    'back',
    'left',
    'front',
    'right',
    'top',
    'bottom'
]


def outImgToXYZ(i, j, faceID, faceWidth, faceHeight):
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


print('Load equirectanglar image...')
equi = HDR.load_from_file(options.input)

for c in range(len(cubemap_names)):
    cm_name = cubemap_names[c]
    print('Generate %s cubemap image...' % cm_name)
    cm_filename = options.input.replace('.hdr', '_' + cm_name + '.hdr')
    cubemap = HDR.create_black_image(options.width, options.height)
    for i in range(options.height):
        for j in range(options.width):
            (x, y, z) = outImgToXYZ(i, j, c, options.width, options.height)

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
