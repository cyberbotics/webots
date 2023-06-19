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

# References:
# - http://paulbourke.net/dataformats/pic/
# - https://github.com/plepers/hdr2png/blob/master/hdrloader.cpp
# - https://github.com/enkimute/hdrpng.js/blob/master/hdrpng.js

import math
import re
import sys
from utils.range import clamp_int

assert sys.version_info >= (3, 0), 'Python 3 is required to run this script.'

GAMMA = 2.0


class HDR:
    @classmethod
    def load_from_file(cls, filename):
        """Parse the HDR file."""
        # HDR Format Specifications: http://paulbourke.net/dataformats/pic/
        #
        # Typical header:
        #     #?RADIANCE
        #     SOFTWARE=gegl 0.4.12
        #     FORMAT=32-bit_rle_rgbe
        #
        #     -Y 1024 +X 2048
        #     Data
        hdr = HDR()
        data = []
        header = False
        with open(filename, "rb") as f:
            while True:
                line = ''
                c = f.read(1).decode('ascii')
                while c != '\n':
                    line += c
                    c = f.read(1).decode('ascii')

                # Case: Empty lines
                if line == '' or (len(line) == 1 and ord(line[0]) == 10):
                    continue
                # Case: header
                m = re.match(r'^#\?RADIANCE$', line)
                if m:
                    header = True
                    continue
                # Case: Size
                m = re.match(r'^(.)(.)\s(\d+)\s(.)(.)\s(\d+)$', line)
                if m:
                    hdr.rotated = m.group(2) == 'X'
                    hdr.xFlipped = m.group(1 if hdr.rotated else 4) == '-'
                    hdr.yFlipped = m.group(4 if hdr.rotated else 1) == '+'
                    hdr.width = int(m.group(6))
                    hdr.height = int(m.group(3))
                    break
                # Case: ignored header entries
                if line.startswith('FORMAT=') or \
                        line.startswith('EXPOSURE=') or \
                        line.startswith('COLORCORR=') or \
                        line.startswith('SOFTWARE=') or \
                        line.startswith('PIXASPECT=') or \
                        line.startswith('VIEW=') or \
                        line.startswith('PRIMARIES=') or \
                        line.startswith('GAMMA=') or \
                        line.startswith('# '):
                    continue
                break
            # Case: Data
            data = f.read()

        assert header, 'Invalid header.'
        assert 4 * hdr.width * hdr.height == len(data) and len(data) > 0, \
            'Invalid dimensions (expected dimension: 4x%dx%d, get %d floats)' % (hdr.width, hdr.height, len(data))
        assert not (hdr.rotated or hdr.xFlipped or hdr.yFlipped), 'Flip or rotation flags are not supported.'

        # Convert data to floats
        hdr.data = [0.0] * (3 * hdr.width * hdr.height)
        for i in range(hdr.width * hdr.height):
            r = float(data[4 * i])
            g = float(data[4 * i + 1])
            b = float(data[4 * i + 2])
            e = pow(2.0, float(data[4 * i + 3]) - 128.0 + 8.0)
            hdr.data[3 * i] = pow(r * e, 1.0 / GAMMA) / 255.0
            hdr.data[3 * i + 1] = pow(g * e, 1.0 / GAMMA) / 255.0
            hdr.data[3 * i + 2] = pow(b * e, 1.0 / GAMMA) / 255.0

        return hdr

    @classmethod
    def create_black_image(cls, width, height):
        """Create an HDR black image."""
        hdr = HDR()
        hdr.width = width
        hdr.height = height
        hdr.data = [0.0] * (3 * hdr.width * hdr.height)
        return hdr

    def __init__(self):
        """Constructor: simply reset the fields. Prefer the static methods."""
        self.data = []  # Contains the 1D array of floats (size: 3*w*h, black: 0.0, white: 1.0, hdr: >1.0)
        self.width = -1
        self.height = -1

        self.xFlipped = False
        self.yFlipped = False
        self.rotated = False

    def is_valid(self):
        """Return True if the image has been loaded correctly."""
        return 3 * self.width * self.height == len(self.data)

    def get_pixel(self, x, y):
        """Get pixel at the speficied position."""
        assert x >= 0 and x < self.width
        assert y >= 0 and y < self.height
        i = 3 * (y * self.width + x)
        return (
            self.data[i],
            self.data[i + 1],
            self.data[i + 2]
        )

    def set_pixel(self, x, y, pixel):
        """Set pixel at the speficied position."""
        assert x >= 0 and x < self.width
        assert y >= 0 and y < self.height
        i = 3 * (y * self.width + x)
        self.data[i] = pixel[0]
        self.data[i + 1] = pixel[1]
        self.data[i + 2] = pixel[2]

    def clamp(self, threshold):
        """Clamp all the floats to some value."""
        assert self.is_valid()
        t = pow(threshold, 1.0 / GAMMA)
        for i in range(3 * self.width * self.height):
            self.data[i] = t if self.data[i] > t else self.data[i]

    def save(self, filename):
        """Save the image to a file."""
        assert self.is_valid()
        assert filename.endswith('.hdr')
        assert not (self.rotated or self.xFlipped or self.yFlipped), 'Flip or rotation flags are not supported.'

        with open(filename, "wb") as f:
            f.write('#?RADIANCE\n'.encode('ascii'))
            f.write('FORMAT=32-bit_rle_rgbe\n'.encode('ascii'))
            f.write('\n'.encode('ascii'))
            f.write(('-Y %d +X %d\n' % (self.height, self.width)).encode('ascii'))
            for i in range(self.width * self.height):
                r = pow(self.data[3 * i], GAMMA)
                g = pow(self.data[3 * i + 1], GAMMA)
                b = pow(self.data[3 * i + 2], GAMMA)
                v = max(r, g, b)
                e = math.ceil(math.log(v, 2)) if v != 0.0 else 0.0
                s = pow(2, e - 8)
                arr = [
                    clamp_int(r / s, 0, 255),
                    clamp_int(g / s, 0, 255),
                    clamp_int(b / s, 0, 255),
                    clamp_int(e + 128, 0, 255)
                ]
                f.write(bytes(arr))

    def to_pil(self):
        """Create a PIL image to test the script."""
        assert self.is_valid()
        from PIL import Image
        im = Image.new('RGB', (self.width, self.height))
        pixels = im.load()
        for y in range(self.height):
            for x in range(self.width):
                i = 3 * (y * self.width + x)
                r = clamp_int(255.0 * self.data[i], 0, 255)
                g = clamp_int(255.0 * self.data[i + 1], 0, 255)
                b = clamp_int(255.0 * self.data[i + 2], 0, 255)
                pixels[x, y] = (r, g, b)
        return im
