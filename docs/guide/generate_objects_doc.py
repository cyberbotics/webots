#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Copyright 1996-2023 Cyberbotics Ltd.
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

import os
from io import open

import sys
sys.path.append('../')
from generate_doc_util import get_proto_files_in_directory, generate_doc_for_proto_files  # noqa: E402


def get_category(proto_file):
    category = os.path.basename(os.path.dirname(os.path.dirname(proto_file)))
    upperCategory = os.path.basename(os.path.dirname(os.path.dirname(os.path.dirname(proto_file))))
    return category, upperCategory


upperCategories = {'projects': ['appearances']}

# look for all the PROTO files in the 'projects/objects' directory
fileList = get_proto_files_in_directory(os.path.join(os.environ['WEBOTS_HOME'], 'projects', 'objects'))
fileList += get_proto_files_in_directory(os.path.join(os.environ['WEBOTS_HOME'], 'projects', 'appearances'))
fileList = sorted(fileList)

# create the 'appearances' page
with open('appearances.md', 'w', encoding='utf-8', newline='\n') as file:
    file.write(u'# Appearances\n')
    file.write(u'This chapter describes the list of available appearance PROTO nodes based on the '
               '[PBRAppearance](../reference/pbrappearance.md) node.\n\n')

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
    upperCategories,
    'object-',
    get_category
)

# write the menu in 'object.md'
del upperCategories['projects']
upperCategoriesList = sorted(upperCategories.keys())
categoriesList = []
with open('objects.md', 'w', encoding='utf-8', newline='\n') as file:
    file.write(u'# Objects\n\n')
    file.write(u'## Sections\n\n')
    for upperCategory in upperCategoriesList:
        categories = sorted(upperCategories[upperCategory])
        if not upperCategory == categories[0]:
            file.write(u'- [%s](object-%s.md)\n' % (upperCategory.replace('_', ' ').title(), upperCategory.replace('_', '-')))
        for category in categories:
            categoriesList.append(category)
            if upperCategory == category:
                file.write(u'- [%s](object-%s.md)\n' % (category.replace('_', ' ').title(), category.replace('_', '-')))
            else:
                file.write(u'  - [%s](object-%s.md#%s)\n' % (category.replace('_', ' ').title(),
                                                             upperCategory.replace('_', '-'), category.replace('_', '-')))
    file.write(u'\n')

# print the updated part of 'menu.md'
categoriesList = sorted(categoriesList)
print("Please update the 'Objects' part in 'menu.md' with:")
for category in upperCategoriesList:
    print('    - [%s](object-%s.md)' % (category.replace('_', ' ').title(), category.replace('_', '-')))
