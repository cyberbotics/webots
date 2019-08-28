# References:
# - http://paulbourke.net/dataformats/pic/
# - https://github.com/plepers/hdr2png/blob/master/hdrloader.cpp
# - https://github.com/enkimute/hdrpng.js/blob/master/hdrpng.js

import re
from PIL import Image


class HDR:
    def __init__(self):
        """Constructor: simply reset the fields."""
        self.data = []
        self.width = -1
        self.height = -1

        self.xFlipped = False
        self.yFlipped = False
        self.rotated = False

    def is_valid(self):
        return 3 * self.width * self.height == len(self.data)

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
            line = True
            while line:
                line = f.readline()

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
                    continue
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
            data = line + f.read()
        assert header, 'Invalid header'
        assert 4 * hdr.width * hdr.height == len(data) and len(data) > 0, 'Invalid dimensions'
        assert not (hdr.rotated or hdr.xFlipped or hdr.yFlipped), 'flip or rotation flags are not supported yet'

        # Convert data to floats
        hdr.data = [0.0] * (3 * hdr.width * hdr.height)
        one_over_gamma = 1.0 / 2.0
        for i in range(hdr.width * hdr.height):
            r = float(ord(data[4 * i]))
            g = float(ord(data[4 * i + 1]))
            b = float(ord(data[4 * i + 2]))
            e = pow(2.0, float(ord(data[4 * i + 3])) - 128.0 + 8.0)
            hdr.data[3 * i] = pow(r * e, one_over_gamma)
            hdr.data[3 * i + 1] = pow(g * e, one_over_gamma)
            hdr.data[3 * i + 2] = pow(b * e, one_over_gamma)

        return hdr

    def to_pil(self):
        """Create a PIL image to test the script."""
        if not self.is_valid():
            return
        im = Image.new('RGB', (self.width, self.height))
        pixels = im.load()
        for y in range(self.height):
            for x in range(self.width):
                index = 3 * (y * self.width + x)
                r = max(0, min(255, int(self.data[index])))
                g = max(0, min(255, int(self.data[index + 1])))
                b = max(0, min(255, int(self.data[index + 2])))
                pixels[x, y] = (r, g, b)
        return im
