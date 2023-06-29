#!/usr/bin/env python
# -*- coding: utf-8 -*-

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

import os
from io import open
import sys
sys.path.append('../')
from generate_doc_util import get_proto_files_in_directory, generate_doc_for_proto_files  # noqa: E402


def get_category(proto_file):
    category = os.path.basename(os.path.dirname(proto_file))
    return category, 'vehicles'


# look for all the PROTO files
vehiclesDir = os.path.join(os.environ['WEBOTS_HOME'], 'projects', 'vehicles', 'protos')
carsDirNames = ['bmw', 'citroen', 'lincoln', 'mercedes_benz', 'range_rover', 'tesla', 'toyota', 'generic']
fileList = []
for dirName in carsDirNames:
    fileList += get_proto_files_in_directory(os.path.join(vehiclesDir, dirName))
fileList = sorted(fileList)

# make sure that if a PROTO has the same name than the title it appears first
prioritaryProtoList = []
for proto in fileList:
    protoName = os.path.basename(proto).split('.')[0]
    categoryName = os.path.basename(os.path.dirname(os.path.dirname(proto)))
    upperCategoryName = os.path.basename(os.path.dirname(os.path.dirname(os.path.dirname(proto))))
    if protoName.lower() == categoryName and upperCategoryName == 'objects':
        prioritaryProtoList.append(proto)
        fileList.remove(proto)

upperCategories = generate_doc_for_proto_files(
    prioritaryProtoList + fileList,
    {},
    'vehicle-',
    get_category
)

# write the menu in 'object.md'
upperCategoriesList = sorted(upperCategories.keys())
categoriesList = []
with open('proto-nodes.md', 'w', encoding='utf-8', newline='\n') as file:
    file.write(u'# PROTO Nodes\n\n')
    file.write(u'This section presents the set of PROTO nodes developed specifically for automobile related simulations, ' +
               'from the PROTO of a wheel to the PROTO of a complete car.\n\n')
    file.write(u'## Sections\n\n')
    file.write(u'- [AckermannVehicle](ackermannvehicle.md)\n')
    file.write(u'- [VehicleWheel](vehiclewheel.md)\n')
    file.write(u'- [Car](car.md)\n')
    for upperCategory in upperCategoriesList:
        categories = sorted(upperCategories[upperCategory])
        for category in categories:
            categoriesList.append(category)
            file.write(u'- [%s](vehicle-%s.md)\n' % (category.replace('_', ' ').title(), category.replace('_', '-')))
    file.write(u'\n')

# print the updated part of 'menu.md'
categoriesList = sorted(categoriesList)
print("Please update the 'PROTO Nodes' part in 'menu.md' with:")
print('    - [AckermannVehicle](ackermannvehicle.md)')
print('    - [VehicleWheel](vehiclewheel.md)')
print('    - [Car](car.md)')
print('    - [Car](car.md)')
for category in categoriesList:
    print('    - [%s](vehicle-%s.md)' % (category.replace('_', ' ').title(), category.replace('_', '-')))
