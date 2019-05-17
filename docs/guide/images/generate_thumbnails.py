#!/usr/bin/env python
# -*- coding: utf-8 -*-

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

import glob
import os
import fnmatch

from PIL import Image

pathes = {
    'samples': (256, 256),
    'appearances': (512, 512),
}

skipped = [
    'samples/mybot.png'
]

images = []

# list images from 'pathes'
for path, size in pathes.items():
    for image in glob.glob(path + os.sep + "*.png"):
        if image not in skipped:
            images.append([image, size])

# specific robot worlds case
for rootPath, dirNames, fileNames in os.walk('robots'):
    for fileName in fnmatch.filter(fileNames, '*.wbt.png'):
        images.append([os.path.join(rootPath, fileName), (256, 256)])

for image in images:
    print('Generating thumbnail(%dx%d) for: %s' % (image[1][0], image[1][1], image[0]))
    im = Image.open(image[0])
    im.thumbnail(image[1])
    im.convert('RGB').save(image[0].replace('.png', '.thumbnail.jpg'), 'JPEG')
