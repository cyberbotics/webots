#!/usr/bin/env python

# Copyright 1996-2022 Cyberbotics Ltd.
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

"""Generate known proto list"""
import os
import glob
import sys
import re
from pathlib import Path
import xml.etree.ElementTree as ET
import xml.dom.minidom

SKIPPED_PROTO = ['UsageProfile.proto']

# ensure WEBOTS_HOME is set
if 'WEBOTS_HOME' in os.environ:
    WEBOTS_HOME = os.environ['WEBOTS_HOME']
else:
    raise RuntimeError('Error, WEBOTS_HOME variable is not set.')

# if no argument is provided, assume it's a local file 'webots://'
if len(sys.argv) == 1:
    base_url = 'webots://'
elif len(sys.argv) == 2:
    base_url = f'https://raw.githubusercontent.com/cyberbotics/webots/{sys.argv[1]}/'
else:
    raise RuntimeError('Unknown argument provided.')

# determine webots version (tag or commit)
with open(os.path.join(WEBOTS_HOME, 'resources', 'version.txt'), 'r') as file:
    VERSION = file.readline().strip()


def parse_proto_header(path):
    license = None
    license_url = None
    documentation_url = None
    description = ''
    tags = []

    # TODO: ensure all proto headers are like this, with space between # and ...

    with open(path, 'r') as file:
        lines = file.readlines()

        for line in lines:
            if not line.startswith('#'):
                break

            if line.startswith('#VRML_SIM') or line.startswith('# template language:'):
                continue
            elif line.startswith('# license:'):
                license = line.replace('# license:', '').strip()
            elif line.startswith('# license url:'):
                license_url = line.replace('# license url:', '').strip()
            elif line.startswith('# documentation url:'):
                documentation_url = line.replace('# documentation url:', '').strip()
            elif line.startswith('# tags:'):
                tags = line.replace('# tags:', '').strip().split(',')
                tags = [tag.strip() for tag in tags]
            else:
                description += line[1:].strip() + ' '

    return license, license_url, description.strip(), tags, documentation_url


if __name__ == "__main__":
    # retrieve all proto and remove exceptions
    assets = []
    assets.extend(Path(WEBOTS_HOME + '/projects').rglob('*.proto'))
    assets = [proto for proto in assets if not any(x in str(proto) for x in SKIPPED_PROTO)]

    # first pass: extract child node from proto itself
    proto_base_nodes = {}
    for proto in assets:
        with open(proto, 'r') as file:
            contents = file.read()

            #print(proto.stem + ' ', end='')
            #child_node = re.search("(?<=[^EXTERN])PROTO\s+[^\[]+\s+\[((.|\n|\r\n)*)\]\s*\n*[^\{]?\{\s*(\%\<((.|\n|\r\n)*)\>\%)?\n?\s*[DEF\s+[a-zA-Z0-9\_\-\+]*]?\s+([a-zA-Z]+)[\s.\n]*{", contents)
            child_node = re.search(
                "(?:\]\s*\n*)\{\n*\s*(?:\%\<((.|\n|\r\n)*)\>\%)?\n*\s*(?:DEF\s+[^\s]+)?\s+([a-zA-Z0-9\_\-\+]+)\s*\{", contents)
            # print(child_node.groups()[-1])
            proto_base_nodes[proto.stem] = child_node.groups()[-1]

    # second pass: determine actual base node by traversing hierarchy until a valid base node is reached
    valid_base_nodes = [os.path.splitext(os.path.basename(x))[0] for x in glob.glob(f'{WEBOTS_HOME}/resources/nodes/*.wrl')]
    for key, value in proto_base_nodes.items():
        while value not in valid_base_nodes:
            if value not in proto_base_nodes:
                raise RuntimeError(f'Error: "{value}" node does not exist. Invalid captures must be occurring in the regex.')
            value = proto_base_nodes[value]
            proto_base_nodes[key] = value

    # generate xml of proto list
    root = ET.Element('proto-list')
    root.set('version', VERSION)

    for proto in assets:
        name = proto.stem
        complete_url = str(proto).replace(WEBOTS_HOME + '/', base_url)
        base_node = proto_base_nodes[proto.stem]
        license, license_url, description, tags, documentation_url = parse_proto_header(proto)

        proto_element = ET.SubElement(root, 'proto')
        name_element = ET.SubElement(proto_element, 'name').text = name
        base_node_element = ET.SubElement(proto_element, 'basenode').text = base_node
        url_element = ET.SubElement(proto_element, 'url').text = complete_url

        if license is not None:
            license_element = ET.SubElement(proto_element, 'license').text = license
        if license_url is not None:
            license_url_element = ET.SubElement(proto_element, 'license-url').text = license_url
        # if documentation_url is not None:
        #    documentation_element = ET.SubElement(proto_element, 'documentation-url').text = documentation_url
        if description is not None:
            description_element = ET.SubElement(proto_element, 'description').text = description
        if tags:
            tags_element = ET.SubElement(proto_element, 'tags').text = ','.join(tags)

    # beautify xml
    tree = ET.ElementTree(root)
    xml_string = xml.dom.minidom.parseString(ET.tostring(root)).toprettyxml(encoding='utf-8')

    # save to file
    filename = f'{WEBOTS_HOME}/resources/proto-list.xml'
    if (os.path.exists(filename)):
        os.remove(filename)

    with open(filename, 'wb') as file:
        file.write(xml_string)
