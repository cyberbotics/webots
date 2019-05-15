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

size = (256, 256)
pathes = [
    'samples'
]

skipped = [
    'samples/mybot.png'
]

images = []

# list images from 'pathes'
for path in pathes:
    for image in glob.glob(path + os.sep + "*.png"):
        if image not in skipped:
            images.append(image)

# specific robot worlds case
for rootPath, dirNames, fileNames in os.walk('robots'):
    for fileName in fnmatch.filter(fileNames, '*.wbt.png'):
        images.append(os.path.join(rootPath, fileName))

for image in images:
    print('Generating thumbnail for: ' + image)
    im = Image.open(image)
    im.thumbnail(size)
    im.convert('RGB').save(image.replace('.png', '_thumbnail.jpg'), 'JPEG')
