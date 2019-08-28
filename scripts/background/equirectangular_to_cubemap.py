#!/usr/bin/python

"""Convert a equirectanglar HDR image to cubemap HDR images."""

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

cubemap_names = {
    'right',  # +x
    'left',  # -x
    'top',  # +y
    'bottom',  # -y
    'front',  # +z
    'back'  # -z
}

print('Load equirectanglar image...')
equi = HDR.load_from_file(options.input)

for cm_name in cubemap_names:
    print('Generate %s cubemap image...' % cm_name)
    cm_filename = options.input.replace('.hdr', '_' + cm_name + '.hdr')
    cubemap = HDR.create_black_image(options.width, options.height)
    for i in range(options.height):
        for j in range(options.width):
            a = float(i) / options.width
            b = float(j) / options.height
            x = 0.0
            y = 0.0
            z = 0.0
            if cm_name == 'right':
                x = 1.0 - a
                y = 1.0
                z = 1.0 - b
            elif cm_name == 'left':
                x = a - 1.0
                y = -1.0
                z = 1.0 - b
            elif cm_name == 'top':
                x = b - 1.0
                y = a - 1.0
                z = 1.0
            elif cm_name == 'bottom':
                x = 1.0 - b
                y = a - 1.0
                z = -1.0
            elif cm_name == 'front':
                x = 1.0
                y = a - 1.0
                z = 1.0 - b
            elif cm_name == 'back':
                x = -1.0
                y = 1.0 - a
                z = 1.0 - b
            theta = math.atan2(y, x)
            rad = math.sqrt(x * x + y * y)
            phi = math.atan2(z, rad)

            uf = equi.width * (theta + math.pi) / math.pi
            vf = equi.height * (math.pi / 2 - phi) / math.pi
            ui = clamp_int(math.floor(uf), 0, equi.width - 1)
            vi = clamp_int(math.floor(vf), 0, equi.height - 1)

            cubemap.set_pixel(i, j, equi.get_pixel(ui, vi))
    cubemap.save(cm_filename)
