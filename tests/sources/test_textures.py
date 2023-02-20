#!/usr/bin/env python

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

"""Test textures."""
import hashlib
import unittest
import os
import filecmp
import fnmatch
from PIL import Image

duplicatedTextures = [
    'mybot.png',
    'soccer_quarter.jpg',
    'pingpong_logo.jpg',
    'conveyor_belt.png',
    'line.png',
    'floor.png',
    # these textures are duplicated but very small and removing them would complexify the PROTO a lot.
    'small_residential_tower_balcony_base_color.jpg',
    'small_residential_tower_ground_floor_windows_base_color.jpg',
    'residential_building_with_round_front_windows_dark_braun_base_color.jpg',
    'residential_building_with_round_front_frames_dark_braun_base_color.jpg',
    'residential_building_with_round_front_stair_dark_braun_metalness.jpg',
    'residential_building_with_round_front_stair_dark_braun_occlusion.jpg',
    'old_residential_building_roof_braun_black_base_color.jpg',
    'residential_building_with_round_front_stair_dark_braun_roughness.jpg',
    'residential_building_with_round_front_stair_green_base_color.jpg',
    'small_residential_building_windows_medium_grey_base_color.jpg',
    'small_residential_building_wall_light_grey_base_color.jpg',
    'small_residential_building_wall_light_grey_occlusion.jpg',
    'dawn_cloudy_empty_bottom.jpg',
    'noon_stormy_empty_bottom.jpg'
]

duplicatedTexturePaths = [
    'projects/samples/robotbenchmark',  # we don't want to change anything to robotbenchmark
    'projects/objects/buildings/protos/textures/colored_textures'
]
duplicatedTexturePaths = [os.path.normpath(path) for path in duplicatedTexturePaths]


def cmpHash(file1, file2):
    """Compare the hash of two files."""
    hash1 = hashlib.md5()
    with open(file1, 'rb') as f:
        hash1.update(f.read())
        hash1 = hash1.hexdigest()
    hash2 = hashlib.md5()
    with open(file2, 'rb') as f:
        hash2.update(f.read())
        hash2 = hash2.hexdigest()
    return hash1 == hash2


class TestTextures(unittest.TestCase):
    """Unit test of the textures."""

    def setUp(self):
        """Get all the textures to be tested."""
        # 1. Get all the images from projects and resources
        images = []
        for directory in ['projects', 'resources']:
            for rootPath, dirNames, fileNames in os.walk(os.path.join(os.path.normpath(os.environ['WEBOTS_HOME']), directory)):
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
                'docs' in image or
                'icons' in image or
                'libraries' in image or
                'plugins' in image or
                'simulator-sdk' in image or
                os.path.join('resources', 'images') in image or
                os.path.join('resources', 'web') in image or
                os.path.join('resources', 'wren') in image
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
            im.close()

    def test_textures_profile(self):
        """Test that the released textures don't contain an ICC profile."""
        for texture in self.textures:
            im = Image.open(texture)

            # Helpful code: uncomment the following lines to drop the ICC profile from every textures.
            # if im.info.get("icc_profile") is not None:
            #     im.info["icc_profile"] = None
            #     im.save(texture)

            self.assertTrue(
                im.info.get("icc_profile") is None,
                msg='texture "%s" contains an ICC profile' % (texture)
            )
            im.close()

    def test_textures_uniqueness(self):
        """Test that the released textures are unique."""
        toCompare = list(self.textures)  # copy
        for texture in self.textures:
            toCompare.remove(texture)
            if any(path in texture for path in duplicatedTexturePaths):
                continue
            if os.path.basename(texture) in duplicatedTextures:
                continue
            for comparedTexture in toCompare:
                if any(path in comparedTexture for path in duplicatedTexturePaths):
                    continue
                if os.path.basename(comparedTexture) in duplicatedTextures:
                    continue

                self.assertTrue(
                    # filecmp is fast but gives false positive, this is why we check the hash too (if needed)
                    (filecmp.cmp(texture, comparedTexture) and cmpHash(texture, comparedTexture)) is False,
                    msg='texture "%s" and "%s" are equal' % (texture, comparedTexture)
                )


if __name__ == '__main__':
    unittest.main()
