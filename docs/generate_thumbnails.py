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

pathes = [
    ['samples', 'jpg', 256, 256],
    ['appearances', 'png', 512, 512]
]

skipped = [
    'samples/mybot.png'
]

images = []

# list images from 'pathes'
for path in pathes:
    for image in glob.glob(path[0] + os.sep + "*.png"):
        if image not in skipped and '.thumbnail.' not in image:
            images.append([image, path[1], path[2], path[3]])

# specific robot worlds case
for rootPath, dirNames, fileNames in os.walk('robots'):
    for fileName in fnmatch.filter(fileNames, '*.wbt.png'):
        images.append([os.path.join(rootPath, fileName), 'jpg', 256, 256])

for image in images:
    print('Generating thumbnail(%dx%d) for: %s' % (image[2], image[3], image[0]))
    im = Image.open(image[0])
    im.thumbnail((image[2], image[3]))
    if image[1] == 'jpg':
        im.convert('RGB').save(image[0].replace('.png', '.thumbnail.jpg'), 'JPEG')
    else:
        im.save(image[0].replace('.png', '.thumbnail.png'), 'PNG')
