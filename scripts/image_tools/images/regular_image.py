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

from PIL import Image


class RegularImage:
    @classmethod
    def load_from_file(cls, filename):
        """Parse an image."""
        ri = RegularImage()
        ri.im = Image.open(filename)
        ri.width, ri.height = ri.im.size
        return ri

    @classmethod
    def create_black_image(cls, width, height):
        """Create an black image."""
        ri = RegularImage()
        ri.im = Image.new('RGB', (width, height))
        ri.width = width
        ri.height = height
        return ri

    def __init__(self):
        """Constructor: simply reset the fields. Prefer the static methods."""
        self.width = -1
        self.height = -1
        self.im = None

    def is_valid(self):
        """Return True if the image has been loaded correctly."""
        return self.width * self.height > 0 and self.im is not None

    def get_pixel(self, x, y):
        """Get pixel at the speficied position."""
        assert x >= 0 and x < self.width
        assert y >= 0 and y < self.height
        return self.im.getpixel((x, y))

    def set_pixel(self, x, y, pixel):
        """Set pixel at the speficied position."""
        assert x >= 0 and x < self.width
        assert y >= 0 and y < self.height
        self.im.putpixel((x, y), (int(pixel[0]), int(pixel[1]), int(pixel[2])))

    def save(self, filename):
        """Save the image to a file."""
        assert self.is_valid()
        self.im.save(filename)

    def to_pil(self):
        """Get the PIL image."""
        assert self.is_valid()
        return self.im
