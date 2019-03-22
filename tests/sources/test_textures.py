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

"""Test textures."""
import unittest
import os
import fnmatch
from PIL import Image


class TestTextures(unittest.TestCase):
    """Unit test of the textures."""

    def setUp(self):
        """Get all the textures to be tested."""
        # 1. Get all the images from projects and resources
        images = []
        for directory in ['projects', 'resources']:
            for rootPath, dirNames, fileNames in os.walk(os.environ['WEBOTS_HOME'] + os.sep + directory):
                for fileName in fnmatch.filter(fileNames, '*.png'):
                    image = os.path.join(rootPath, fileName)
                    images.append(image)
                for fileName in fnmatch.filter(fileNames, '*.jpg'):
                    image = os.path.join(rootPath, fileName)
                    images.append(image)
        # 2. filter-out the images which are not textures
        self.textures = []
        for image in images:
            if not (
                'controllers' in image or
                'icons' in image or
                'libraries' in image or
                'plugins' in image or
                'simulator-sdk' in image or
                'resources' + os.sep + 'images' in image or
                'resources' + os.sep + 'web' in image or
                'resources' + os.sep + 'wren' in image
            ):
                self.textures.append(image)

    def test_textures_dimensions_are_power_of_two(self):
        """Test that the released textures dimensions are power of two."""
        def is_perfect_power_of_two(a):
            assert isinstance(a, int)
            while a % 2 == 0:
                a = a / 2
            if a == 1:
                return True
            return False

        for texture in self.textures:
            im = Image.open(texture)
            self.assertTrue(
                is_perfect_power_of_two(im.size[0]) and is_perfect_power_of_two(im.size[1]),
                msg='texture "%s": dimension is not a power of two: (%d, %d)' % (texture, im.size[0], im.size[1])
            )

    def test_textures_profile(self):
        """Test that the released textures don't contain an ICC profile."""
        for texture in self.textures:
            im = Image.open(texture)
            self.assertTrue(
                im.info.get("icc_profile") is None,
                msg='texture "%s" contains an ICC profile' % (texture)
            )


if __name__ == '__main__':
    unittest.main()
