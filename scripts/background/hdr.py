import re
from PIL import Image


class HDR:
    def __init__(self, filename):
        """Constructor: simply set the filename."""
        self.filename = filename
        self.reset()

    def reset(self):
        """Reset internal values."""
        self.data = []
        self.width = -1
        self.height = -1

        self.header = False
        self.xFlipped = False
        self.yFlipped = False
        self.rotated = False

    def parse(self):
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

        self.reset()
        with open(self.filename, "rb") as f:
            line = True
            while line:
                line = f.readline()

                # Case: Empty lines
                if line == '':
                    continue
                # Case: header
                m = re.match(r'^#\?RADIANCE$', line, re.IGNORECASE)
                if m:
                    self.header = True
                    continue
                # Case: Size
                m = re.match(r'^(.)(.)\s(\d+)\s(.)(.)\s(\d+)$', line, re.IGNORECASE)
                if m:
                    self.rotated = m.group(2) == 'X'
                    self.xFlipped = m.group(1 if self.rotated else 4) == '-'
                    self.yFlipped = m.group(4 if self.rotated else 1) == '+'
                    self.width = int(m.group(6))
                    self.height = int(m.group(3))
                # Case: Data
                self.data = line
        return self.is_valid()

    def is_valid(self):
        return self.header and 4 * self.width * self.height == len(self.data)

    def to_pil(self):
        if not self.is_valid():
            return
        im = Image.new('RGB', (self.width, self.height))
        pixels = im.load()
        for y in range(self.height):
            for x in range(self.width):
                index = 4 * (y * self.width + x)
                r = float(ord(self.data[index]))
                g = float(ord(self.data[index + 1]))
                b = float(ord(self.data[index + 2]))
                e = float(ord(self.data[index + 3])) - 128.0
                r = min(255, int(r * pow(2.0, e)))
                g = min(255, int(g * pow(2.0, e)))
                b = min(255, int(b * pow(2.0, e)))
                pixels[x, y] = (r, g, b)
        return im
