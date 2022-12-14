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

import fnmatch
import os
import re
import sys
import xml.etree.ElementTree as ET

from io import open

# https://github.com/rcompton/ryancompton.net/blob/released/assets/praw_drugs/urlmarker.py
WEB_URL_REGEX = \
    r'(?i)\b((?:https?:(?:/{1,3}|[a-z0-9%])|[a-z0-9.\-]+[.](?:com|net|org|edu|gov|mil|aero|asia|biz|cat|coop|info|' \
    r'int|jobs|mobi|museum|name|post|pro|tel|travel|xxx|ac|ad|ae|af|ag|ai|al|am|an|ao|aq|ar|as|at|au|aw|ax|az|ba|' \
    r'bb|bd|be|bf|bg|bh|bi|bj|bm|bn|bo|br|bs|bt|bv|bw|by|bz|ca|cc|cd|cf|cg|ch|ci|ck|cl|cm|cn|co|cr|cs|cu|cv|cx|' \
    r'cy|cz|dd|de|dj|dk|dm|do|dz|ec|ee|eg|eh|er|es|et|eu|fi|fj|fk|fm|fo|fr|ga|gb|gd|ge|gf|gg|gh|gi|gl|gm|gn|gp|gq|' \
    r'gr|gs|gt|gu|gw|gy|hk|hm|hn|hr|ht|hu|id|ie|il|im|in|io|iq|ir|is|it|je|jm|jo|jp|ke|kg|kh|ki|km|kn|kp|kr|kw|ky|' \
    r'kz|la|lb|lc|li|lk|lr|ls|lt|lu|lv|ly|ma|mc|md|me|mg|mh|mk|ml|mm|mn|mo|mp|mq|mr|ms|mt|mu|mv|mw|mx|my|mz|na|nc|' \
    r'ne|nf|ng|ni|nl|no|np|nr|nu|nz|om|pa|pe|pf|pg|ph|pk|pl|pm|pn|pr|ps|pt|pw|py|qa|re|ro|rs|ru|rw|sa|sb|sc|sd|se|' \
    r'sg|sh|si|sj|Ja|sk|sl|sm|sn|so|sr|ss|st|su|sv|sx|sy|sz|tc|td|tf|tg|th|tj|tk|tl|tm|tn|to|tp|tr|tt|tv|tw|tz|ua|' \
    r'ug|uk|us|uy|uz|va|vc|ve|vg|vi|vn|vu|wf|ws|ye|yt|yu|za|zm|zw)/)(?:[^\s()<>{}\[\]]+|\([^\s()]*?\([^\s()]+\)' \
    r'[^\s()]*?\)|\([^\s]+?\))+(?:\([^\s()]*?\([^\s()]+\)[^\s()]*?\)|\([^\s]+?\)|[^\s`!()\[\]{};:\'".,<>?«»“”‘’])|' \
    r'(?:(?<!@)[a-z0-9]+(?:[.\-][a-z0-9]+)*[.](?:com|net|org|edu|gov|mil|aero|asia|biz|cat|coop|info|int|jobs|' \
    r'mobi|museum|name|post|pro|tel|travel|xxx|ac|ad|ae|af|ag|ai|al|am|an|ao|aq|ar|as|at|au|aw|ax|az|ba|bb|bd|be|' \
    r'bf|bg|bh|bi|bj|bm|bn|bo|br|bs|bt|bv|bw|by|bz|ca|cc|cd|cf|cg|ch|ci|ck|cl|cm|cn|co|cr|cs|cu|cv|cx|cy|cz|dd|de|' \
    r'dj|dk|dm|do|dz|ec|ee|eg|eh|er|es|et|eu|fi|fj|fk|fm|fo|fr|ga|gb|gd|ge|gf|gg|gh|gi|gl|gm|gn|gp|gq|gr|gs|gt|gu|' \
    r'gw|gy|hk|hm|hn|hr|ht|hu|id|ie|il|im|in|io|iq|ir|is|it|je|jm|jo|jp|ke|kg|kh|ki|km|kn|kp|kr|kw|ky|kz|la|lb|lc|' \
    r'li|lk|lr|ls|lt|lu|lv|ly|ma|mc|md|me|mg|mh|mk|ml|mm|mn|mo|mp|mq|mr|ms|mt|mu|mv|mw|mx|my|mz|na|nc|ne|nf|ng|ni|' \
    r'nl|no|np|nr|nu|nz|om|pa|pe|pf|pg|ph|pk|pl|pm|pn|pr|ps|pt|pw|py|qa|re|ro|rs|ru|rw|sa|sb|sc|sd|se|sg|sh|si|sj|' \
    r'Ja|sk|sl|sm|sn|so|sr|ss|st|su|sv|sx|sy|sz|tc|td|tf|tg|th|tj|tk|tl|tm|tn|to|tp|tr|tt|tv|tw|tz|ua|ug|uk|us|uy|' \
    r'uz|va|vc|ve|vg|vi|vn|vu|wf|ws|ye|yt|yu|za|zm|zw)\b/?(?!@)))'

DESCRIPTION_STATE = 0
FIELDS_STATE = 1
BODY_STATE = 2
TAG = 'R2023a'

# ensure WEBOTS_HOME is set and tag was provided
if 'WEBOTS_HOME' in os.environ:
    WEBOTS_HOME = os.path.normpath(os.environ['WEBOTS_HOME'])
else:
    raise RuntimeError('Error, WEBOTS_HOME variable is not set.')


def get_proto_files_in_directory(directory):
    fileList = []
    for rootPath, dirNames, fileNames in os.walk(directory):
        for fileName in fnmatch.filter(fileNames, '*.proto'):
            fileList.append(os.path.join(rootPath, fileName))
    return fileList


def generate_doc_for_proto_files(proto_files, upperCategories, file_prefix, get_category):
    # get list of base nodes
    baseNodeList = []
    for rootPath, dirNames, fileNames in os.walk(os.path.join(WEBOTS_HOME, 'resources', 'nodes')):
        for fileName in fnmatch.filter(fileNames, '*.wrl'):
            baseNodeList.append(os.path.splitext(fileName)[0])

    # loop through all PROTO files
    for proto in proto_files:
        protoName = os.path.basename(proto).split('.')[0]
        category, upperCategory = get_category(proto)
        if upperCategory == 'objects':
            upperCategory = category
        upperCategoryName = upperCategory.replace('_', '-')
        categoryName = category.replace('_', '-')
        description = ''
        license = ''
        licenseUrl = ''
        fields = u''
        describedField = []
        fieldEnumeration = {}
        skipProto = False
        # parse the PROTO file
        with open(proto, 'r', encoding='utf-8') as file:
            content = file.read()
            # header
            matches = re.finditer(r'^#.*\n', content, re.MULTILINE)
            for i, match in enumerate(matches):
                line = match.group()
                if line.startswith('#VRML_SIM'):
                    continue
                if line.startswith('# template language'):
                    continue
                elif line.startswith('# license:'):
                    license = line.replace('# license:', '').strip()
                elif line.startswith('# license url:'):
                    licenseUrl = line.replace('# license url:', '').strip()
                elif line.startswith('# documentation url:'):
                    continue
                elif line.startswith('# tags:'):
                    if 'deprecated' in line or 'hidden' in line:
                        skipProto = True
                        break
                else:
                    newLine = line.replace('#', '').replace('_', '\\_').strip()
                    urls = re.findall(WEB_URL_REGEX, newLine)
                    for url in urls:
                        newLine = newLine.replace(url, '[%s](%s)' % (url, url))
                    description += newLine + '\n'
            if skipProto:
                if upperCategory == 'projects':
                    imagePath = os.path.join('images', category, protoName + '.png')
                elif upperCategory == 'vehicles':
                    imagePath = os.path.join('images', category, protoName, 'model.png')
                else:
                    imagePath = os.path.join('images', upperCategory, category, protoName, 'model.png')
                if os.path.exists(imagePath):
                    os.remove(imagePath)
                thumbnailPath = imagePath.replace('.png', '.thumbnail.png')
                if os.path.exists(thumbnailPath):
                    os.remove(thumbnailPath)
                continue
            # fields
            matches = re.finditer(r'\[\n((.*\n)*)\]', content, re.MULTILINE)
            for i, match in enumerate(matches):
                fieldsDefinition = match.group(1)
                break  # only first match is interesting
            # remove enumerations
            matches = re.finditer(r'.*ield\s+([^ ]*?)(\{(?:[^\[\n]*\,?\s?)(?<!(\{))\})\s+([^ ]*)\s+([^#\n]*)(#?)(.*)',
                                  fieldsDefinition, re.MULTILINE)
            for i, match in enumerate(matches):
                fieldEnumeration[match.group(4)] = match.group(2)[1:-1].split(',')  # keep the list of possibilities
                if '\n' in match.group():
                    string = ' ' * match.group().index(match.group(2))
                    fieldsDefinition = fieldsDefinition.replace(string + match.group(4), match.group(4))
                    fieldsDefinition = fieldsDefinition.replace(match.group(2) + '\n', '')
                if len(match.group(2)) < 40:
                    fieldsDefinition = fieldsDefinition.replace(match.group(2), ' ' * len(match.group(2)))
                else:
                    fieldsDefinition = fieldsDefinition.replace(match.group(2), '')
            # count minimum space number between field type and name
            matches = re.finditer(r'.*ield\s+([^ ]*?)(\s+)([^ ]*)\s+([^#\n]*)(#?)(.*)',
                                  fieldsDefinition, re.MULTILINE)
            minSpaces = 2000
            for i, match in enumerate(matches):
                spaces = match.group(2)
                if len(spaces) < minSpaces:
                    minSpaces = len(spaces)
            spacesToRemove = max(minSpaces - 2, 0)
            # create the final cleaned PROTO header
            matches = re.finditer(r'^\s*(.*?ield)\s+([^ \{]*)(\s+)([^ ]*)\s+([^#\n]*)(#?)(.*)((\n*(    |  \]).*)*)',
                                  fieldsDefinition, re.MULTILINE)
            for i, match in enumerate(matches):
                if match.group(1) not in ['hiddenField', 'deprecatedField']:
                    fieldType = match.group(2)
                    spaces = match.group(3)
                    fieldName = match.group(4)
                    fieldComment = match.group(7).strip()
                    # skip 'Is `NodeType.fieldName`.' descriptions
                    if fieldComment and not re.match(r'Is\s`([a-zA-Z]*).([a-zA-Z]*)`.', fieldComment):
                        # add link to base nodes:
                        for baseNode in baseNodeList:
                            link = ' [' + baseNode + '](../reference/' + baseNode.lower() + '.md)'
                            fieldComment = fieldComment.replace(' ' + baseNode, link)
                        describedField.append([fieldType, fieldName, fieldComment])
                    # remove the comment
                    fieldString = match.group()
                    fieldString = re.sub(r'\s*(#.*)', '', match.group(), 0, re.MULTILINE)
                    # remove intial '*field' string
                    fieldString = re.sub(r'^\s*.*field\s', '  ', fieldString, 0, re.MULTILINE)
                    # update urls
                    fieldString = fieldString.replace(
                        'webots://', 'https://raw.githubusercontent.com/cyberbotics/webots/' + TAG + '/')
                    # remove unwanted spaces between field type and field name (if needed)
                    if spacesToRemove > 0:
                        fieldString = fieldString.replace(fieldType + ' ' * spacesToRemove, fieldType)
                    fields += fieldString + '\n'

        baseType = None
        # use the proto-list.xml file to get the baseType
        root = ET.parse(os.path.join(WEBOTS_HOME, 'resources', 'proto-list.xml')).getroot()

        for child in root:
            for item in list(child):
                if item.tag == 'name' and item.text == protoName:
                    baseType = child.find('base-type').text
        if baseType is None:
            sys.stderr.write(f'Could not find proto \"{protoName}\"\n')

        # add documentation for this PROTO file
        if upperCategory == 'projects':
            fileName = 'appearances.md'
            mode = 'a'
        elif upperCategory == 'vehicles':
            fileName = file_prefix + categoryName + '.md'
            mode = 'w' if upperCategory not in upperCategories or category not in upperCategories[upperCategory] else 'a'
        else:
            fileName = file_prefix + upperCategoryName + '.md'
            mode = 'w' if upperCategory not in upperCategories else 'a'
        with open(fileName, mode, encoding='utf-8', newline='\n') as file:
            if upperCategory not in upperCategories and upperCategory not in [category, 'projects', 'vehicles']:
                file.write(u'# %s\n\n' % upperCategory.replace('_', ' ').title())
            headerPrefix = u'#'
            if upperCategory not in [category, 'projects', 'vehicles']:
                headerPrefix = u'##'

            if upperCategory not in upperCategories or category not in upperCategories[upperCategory]:
                file.write(headerPrefix + u' %s\n\n' % category.replace('_', ' ').title())
            if protoName not in [upperCategory.replace('_', ' ').title(), category.replace('_', ' ').title()]:
                file.write(headerPrefix + u'# %s\n\n' % protoName)
            else:
                # Avoid writing twice the same title at different level
                file.write(headerPrefix + u'# %s PROTO\n\n' % protoName)

            file.write(description + '\n')

            if upperCategory == 'projects':  # appearances
                imagePath = os.path.join('images', category, protoName + '.png')
            elif upperCategory == 'vehicles':
                imagePath = os.path.join('images', category, protoName + '.png')
            else:
                imagePath = os.path.join('images', 'objects', category, protoName, 'model.png')
            thumbnailPath = imagePath.replace('.png', '.thumbnail.png')
            if os.path.isfile(thumbnailPath):
                imagePath = thumbnailPath
            if os.path.isfile(imagePath):
                file.write(u'%figure\n\n')
                file.write(u'![%s](%s)\n\n' % (protoName, imagePath))
                file.write(u'%end\n\n')
            else:
                # maybe multiple images
                if os.path.exists(os.path.dirname(imagePath)):
                    availableImages = [os.path.join(os.path.dirname(imagePath), f) for f in
                                       os.listdir(os.path.dirname(imagePath))]
                    regex = imagePath.replace('.png', '_..png')
                    files = []
                    for image in availableImages:
                        if re.match(regex, image):
                            thumbnailPath = image.replace('.png', '.thumbnail.png')
                            if os.path.isfile(thumbnailPath):
                                image = thumbnailPath
                            files.append(image)
                    if files:
                        files.sort()  # alphabetically ordered
                        file.write(u'%figure\n\n')
                        file.write(u'|     |     |\n')
                        file.write(u'|:---:|:---:|\n')
                        for i in range(len(files)):
                            image = os.path.basename(files[i]).replace('.thumbnail.png', '.png')
                            if i % 2 == 0:
                                file.write(u'| ![%s](%s) |' % (image, files[i]))
                            else:
                                file.write(u'![%s](%s) |\n' % (image, files[i]))
                        if not len(files) % 2 == 0:
                            file.write(u' |\n')
                        file.write(u'\n%end\n\n')
                    else:
                        sys.stderr.write('Please add a "%s" file.\n' % imagePath)
                else:
                    sys.stderr.write('Please add a "%s" file.\n' % imagePath)

            if baseType:
                file.write(u'Derived from [%s](../reference/%s.md).\n\n' % (baseType, baseType.lower()))

            file.write(u'```\n')
            file.write(u'%s {\n' % protoName)
            file.write(fields)
            file.write(u'}\n')
            file.write(u'```\n\n')
            location = os.path.normpath(proto.replace(WEBOTS_HOME, ''))
            file.write(u'> **File location**: "[WEBOTS\\_HOME%s]({{ url.github_tree }}%s)"\n\n' %
                       (location.replace('_', '\\_'), location))
            if license:
                file.write(u'> **License**: %s\n' % license)
                if licenseUrl:
                    file.write(u'[More information.](%s)\n' % licenseUrl)
                file.write(u'\n')
            else:
                sys.stderr.write('Please add a license to "%s"\n' % proto)

            if describedField:
                file.write(headerPrefix + u'## %s Field Summary\n\n' % protoName)
                for fieldType, fieldName, fieldDescription in describedField:
                    file.write(u'- `%s`: %s' % (fieldName, fieldDescription))
                    isMFField = fieldType.startswith('MF')
                    if fieldName in fieldEnumeration:
                        values = fieldEnumeration[fieldName]
                        if isMFField:
                            file.write(u' This field accepts a list of ')
                        else:
                            if len(values) > 1:
                                file.write(u' This field accepts the following values: ')
                            else:
                                file.write(u' This field accepts the following value: ')
                        for i in range(len(values)):
                            value = values[i].split('{')[0]  # In case of node keep only the type
                            if i == len(values) - 1:
                                if isMFField:
                                    file.write(u'`%s` %s.' % (value.strip(), fieldType.replace('MF', '').lower() + 's'))
                                else:
                                    file.write(u'`%s`.' % value.strip())
                            elif i == len(values) - 2:
                                if len(values) == 2:
                                    file.write(u'`%s` and ' % value.strip())
                                else:
                                    file.write(u'`%s`, and ' % value.strip())
                            else:
                                file.write(u'`%s`, ' % value.strip())
                    file.write(u'\n\n')

        if upperCategory not in upperCategories:
            upperCategories[upperCategory] = []
            upperCategories[upperCategory].append(category)
        elif category not in upperCategories[upperCategory]:
            upperCategories[upperCategory].append(category)

    return upperCategories
