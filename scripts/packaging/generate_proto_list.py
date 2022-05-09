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

class ProtoInfo:
    def __init__(self, path, name):
        self.name = name
        self.path = path
        self.proto_type = None  # direct node type, ex: for RoadSegment is Road
        self.base_type = None   # lowest node type, ex: for RoadSegment is Solid
        self.license = None
        self.license_url = None
        self.description = ''
        self.documentation_url = None # TODO: needed?
        self.tags = []
        # exclusive to slots
        self.slot_type = None

        self.parse_header()
        self.parse_body()

    def parse_header(self):
        # TODO: if space between "license" and ":"?
        with open(self.path, 'r') as file:
            lines = file.readlines()

            for line in lines:
                if not line.startswith('#'):
                    break

                clean_line = line[1:].strip()
                if clean_line.startswith('VRML_SIM') or line.startswith('template language'):
                    continue
                elif clean_line.startswith('license:'):
                    self.license = clean_line.replace('license:', '').strip()
                elif line.startswith('license url:'):
                    self.license_url = clean_line.replace('license url:', '').strip()
                elif line.startswith('documentation url:'):
                    self.documentation_url = clean_line.replace('documentation url:', '').strip()
                elif line.startswith('tags:'):
                    tags = clean_line.replace('tags:', '').strip().split(',')
                    self.tags = [tag.strip() for tag in tags]
                else:
                    self.description += clean_line[1:].strip() + '\n'

    def parse_body(self):
        # determine proto type (base type can be inferred only when all proto_types are known)
        with open(self.path, 'r') as file:
            contents = file.read()

            #print(proto.stem + ' ', end='')
            #child_node = re.search("(?<=[^EXTERN])PROTO\s+[^\[]+\s+\[((.|\n|\r\n)*)\]\s*\n*[^\{]?\{\s*(\%\<((.|\n|\r\n)*)\>\%)?\n?\s*[DEF\s+[a-zA-Z0-9\_\-\+]*]?\s+([a-zA-Z]+)[\s.\n]*{", contents)
            child_node = re.search(
                "(?:\]\s*\n*)\{\n*\s*(?:\%\<((.|\n|\r\n)*)\>\%)?\n*\s*(?:DEF\s+[^\s]+)?\s+([a-zA-Z0-9\_\-\+]+)\s*\{", contents)
            # print(child_node.groups()[-1])
            self.proto_type = child_node.groups()[-1]

    def print(self):
      print(f'{self.name}: {self.path}\n     ({self.proto_type}) -> ({self.base_type}) / {self.slot_type}')

# determine webots version (tag or commit)
with open(os.path.join(WEBOTS_HOME, 'resources', 'version.txt'), 'r') as file:
    VERSION = file.readline().strip()


def parse_proto_body(path, base_node):
    slot_type = None

    if base_node == "Slot":
      print(path)
      with open(path, 'r') as file:
          content = file.read()

          matches = re.findall("type\s+\"([a-zA-Z0-9\_\-\+\s]+)\"", content)
          if len(matches) == 0:
              raise RuntimeError(f'PROTO {path} of base node "Slot" should specify a type but does not. The regex is likely not catching it.')
          else:
              slot_type = matches[0] # we are interested in the first Slot, if multiple exist

    return slot_type

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
                description += line[1:].strip() + '\n'

    return license, license_url, description.strip(), tags, documentation_url


if __name__ == "__main__":
    assets = []
    assets.extend(Path(WEBOTS_HOME + '/projects').rglob('*.proto'))

    protos = {}
    for asset in assets:
      if asset.name in SKIPPED_PROTO:
          continue

      info = ProtoInfo(str(asset), asset.stem)
      if info.name in protos:
          raise RuntimeError(f'PROTO names should be unique, but {info.name} is not.')
      else:
          protos[info.name] = info


    base_nodes = [os.path.splitext(os.path.basename(x))[0] for x in glob.glob(f'{WEBOTS_HOME}/resources/nodes/*.wrl')]
    # determine base_type from proto_type
    for key, value in protos.items():
        value.base_type = value.proto_type # begin by assuming the proto depends directly on a base node
        print(f'{value.name}: checking if {value.base_type} in base nodes')
        if value.base_type in base_nodes:
            continue

        while value.base_type not in base_nodes:
            if value.proto_type in base_nodes:
                value.base_type = value.proto_type
                break
            print(f'  check {value.name} ({value.proto_type})')
            if value.proto_type not in protos:
                raise RuntimeError(f'Error: "{value.proto_type}" proto node does not exist. Either it was skipped or the regex that retrieves the type is incorrect.')
            print(f'  replacing {value.name} with {protos[value.proto_type].name}')
            value = protos[value.proto_type]
            protos[key].base_type = value.base_type

    for key, value in protos.items():
        value.print()
    exit()
    # determine the slot type, if applicable
    for key, info in protos.items():
        if info.base_type == "Slot":
            # iterate through lower level nodes until a valid slot type definition is found
            found = False
            while not found:
                current_proto = info
                with open(current_proto.path, 'r') as file:
                    contents = file.read()
                    matches = re.findall("type\s+\"([a-zA-Z0-9\_\-\+\s]+)\"", file.read())
                    if len(matches) > 0:
                        info.slot_type = matches[0] # we are interested in the first Slot, if multiple exist
                        found = True
                    else:
                        # slot type might be defined on the subproto, if any
                        if current_proto.proto_type not in base_nodes:
                            if current_proto.proto_type in protos:
                                current_proto = protos[current_proto.proto_type]
                            else:
                                raise RuntimeError(f'Sub-proto "{current_proto.proto_type}" is not a known proto.')




    '''
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
        slot_type = parse_proto_body(proto, base_node)
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
        if slot_type:
            slot_element = ET.SubElement(proto_element, 'slot-type').text = slot_type

    # beautify xml
    tree = ET.ElementTree(root)
    xml_string = xml.dom.minidom.parseString(ET.tostring(root)).toprettyxml(encoding='utf-8')

    # save to file
    filename = f'{WEBOTS_HOME}/resources/proto-list.xml'
    if (os.path.exists(filename)):
        os.remove(filename)

    with open(filename, 'wb') as file:
        file.write(xml_string)
    '''