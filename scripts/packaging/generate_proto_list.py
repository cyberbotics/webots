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
import hashlib
import time

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

# list of devices and regex to match if any of them is present
devices = ['Brake', 'LinearMotor', 'PositionSensor', 'RotationalMotor', 'Skin', 'Accelerometer', 'Altimeter', 'Camera',
           'Compass', 'Compass', 'Display', 'DistanceSensor', 'Emitter', 'GPS', 'Gyro', 'InertialUnit', 'LED', 'Lidar',
           'LightSensor', 'Pen', 'Radar', 'RangeFinder', 'Receiver', 'Speaker', 'TouchSensor', 'Track']

regex_device = [rf'\s+{device}\s*' for device in devices]
regex_device = "(" + "|".join(regex_device) + ")"


class ProtoInfo:
    def __init__(self, path, name):
        self.name = name
        self.path = path
        self.proto_type = None  # direct node type, ex: for RoadSegment is Road
        self.base_type = None   # lowest node type, ex: for RoadSegment is Solid
        self.license = None
        self.license_url = None
        self.description = ''
        self.documentation_url = None  # TODO: needed?
        self.tags = []
        # exclusive to slots
        self.slot_type = None
        self.needs_robot_ancestor = False
        # determines if needs to be re
        self.hash = hashlib.sha1(str(self.path).encode('utf-8')).hexdigest()

        # file contents to avoid reading multiple times
        with open(self.path, 'r') as file:
            self.contents = file.read()
            self.lines = self.contents.split('\n')

        self.parse_header()
        self.parse_body()

    def parse_header(self):
        for line in self.lines:
            if not line.startswith('#'):
                break

            clean_line = line[1:].strip()
            if clean_line.startswith('VRML_SIM') or re.search('template language\s*:', clean_line):
                continue
            elif re.search('license\s*:', clean_line):
                self.license = re.sub('license\s*:', '', clean_line).strip()
            elif re.search('license url\s*:', clean_line):
                self.license_url = re.sub('license url\s*:', '', clean_line).strip()
            elif re.search('documentation url\s*:', clean_line):
                self.documentation_url = re.sub('documentation url\s*:', '', clean_line).strip()
            elif re.search('tags\s*:', clean_line):
                tags = re.sub('tags\s*:', '', clean_line).strip().split(',')
                self.tags = [tag.strip() for tag in tags]
            else:
                self.description += clean_line.strip() + '\\n'

    def parse_body(self):
        # determine proto type (base type can be inferred only when all proto_types are known)
        # print(proto.stem + ' ', end='')
        # child_node = re.search("(?<=[^EXTERN])PROTO\s+[^\[]+\s+\[((.|\n|\r\n)*)\]\s*\n*[^\{]?\{\s*(\%\<((.|\n|\r\n)*)\>\%)?\n?\s*[DEF\s+[a-zA-Z0-9\_\-\+]*]?\s+([a-zA-Z]+)[\s.\n]*{", contents)
        # child_node = re.search(
        #    "(?:\]\s*\n*)\{\n*\s*(?:\%\<((.|\n|\r\n)*)\>\%)?\n*\s*(?:DEF\s+[^\s]+)?\s+([a-zA-Z0-9\_\-\+]+)\s*\{", contents)
        child_node = re.search(
            '(?:\]\s*\n*)\{\n*\s*(?:\%\<[\s\S]*?(?:\>\%\n*\s*))?(?:DEF\s+[^\s]+)?\s+([a-zA-Z0-9\_\-\+]+)\s*\{', self.contents)
        # print(child_node.groups()[-1])
        self.proto_type = child_node.groups()[-1]

    def check_robot_ancestor_requirement(self):
        global regex_device
        if self.proto_type in ['Solid', 'Transform', 'Group']:
            # check if contains devices
            if re.search(regex_device, self.contents):
                self.needs_robot_ancestor = True


if __name__ == "__main__":
    assets = []
    assets.extend(Path(WEBOTS_HOME + '/projects').rglob('*.proto'))

    proto_hashes = {}
    # load hash list or generate it if unknown
    hashfile = f'{WEBOTS_HOME}/resources/.proto-list.xml'
    if os.path.exists(hashfile):
        with open(hashfile, 'r') as file:
            lines = file.readlines()
            for line in lines:
                line = line[0:-1].split(' ')
                proto_hashes[line[0]] = line[1]

    protos = {}
    needs_refresh = False  # whether or not proto-list needs to be regenerated
    for asset in assets:
        if asset.name in SKIPPED_PROTO:
            continue

        info = ProtoInfo(str(asset), asset.stem)
        if info.name in protos:
            raise RuntimeError(f'PROTO names should be unique, but {info.name} is not.')
        else:
            protos[info.name] = info
            hash = hashlib.sha1(info.contents.encode('utf-8')).hexdigest()
            if info.name not in proto_hashes or proto_hashes[info.name] != hash:
                needs_refresh = True

    if needs_refresh or len(proto_hashes) != len(protos):
        print('Changes detected, proto-list will be regenerated. The process might take some time.')
    else:
        print('No changes detected, proto-list regeneration will be skipped')
        exit(0)

    base_nodes = [os.path.splitext(os.path.basename(x))[0] for x in glob.glob(f'{WEBOTS_HOME}/resources/nodes/*.wrl')]
    # determine base_type from proto_type
    for key, info in protos.items():
        info.base_type = info.proto_type
        while info.base_type not in base_nodes:
            # the current proto depends on a sub-proto, so iterate over it until a base node is reached
            if info.base_type not in protos:
                raise RuntimeError(
                    f'Error: "{info.base_type}" proto node does not exist. Either it was skipped or the regex that retrieves the type is incorrect.')
            sub_proto = protos[info.base_type]
            info.base_type = sub_proto.proto_type

    # all Solid, Transform and Group nodes might in fact be a collection of devices, so determine if it needs a Robot ancestor
    for key, info in protos.items():
        info.check_robot_ancestor_requirement()

    # order based on proto path
    # protos = sorted(protos.items(), key=lambda x: x[1].path)

    # for key, value in protos.items():
    #     if 'Velodyne' in key:
    #         value.print()

    # determine the slot type, if applicable
    for key, info in protos.items():
        if info.base_type == "Slot":
            # iterate through lower level nodes until a valid slot type definition is found
            found = False
            current_proto = info
            while not found:
                # check if the node defines a type
                matches = re.findall("type\s+\"([a-zA-Z0-9\_\-\+\s]+)\"", current_proto.contents)
                if len(matches) > 0:
                    info.slot_type = matches[0]  # we are interested in the first Slot, if multiple exist
                    found = True
                else:
                    # if this proto depends on a sub-proto, the type might be defined there instead
                    if current_proto.proto_type in base_nodes:
                        if current_proto.proto_type == "Slot":
                            info.slot_type = ''  # default value of the 'type' field in a Slot node
                            found = True
                        else:
                            raise RuntimeError(f'Reached a non-Slot base node. This should not be possible.')
                    else:
                        if current_proto.proto_type in protos:
                            current_proto = protos[current_proto.proto_type]  # go one level deeper
                        else:
                            raise RuntimeError(f'Sub-proto "{current_proto.proto_type}" is not a known proto.')

    # for key, value in protos.items():
    #     if value.base_type == 'Slot':
    #         value.print()

    # generated a sorted list from the dictionary
    # ordered_protos = sorted([x.path for x in protos.values()])

    # generate xml of proto list
    root = ET.Element('proto-list')
    root.set('version', VERSION)

    for key, info in protos.items():
        # retrieve proto meta info from dictionary following the alphabetical order of the paths
        # info = protos[os.path.basename(item).replace(".proto", "")]
        proto_element = ET.SubElement(root, 'proto')
        name_element = ET.SubElement(proto_element, 'name').text = info.name
        base_node_element = ET.SubElement(proto_element, 'basenode').text = info.base_type
        url_element = ET.SubElement(proto_element, 'url').text = info.path.replace(WEBOTS_HOME + '/', base_url)

        if info.license is not None:
            license_element = ET.SubElement(proto_element, 'license').text = info.license
        if info.license_url is not None:
            license_url_element = ET.SubElement(proto_element, 'license-url').text = info.license_url
        if info.documentation_url is not None:
           documentation_element = ET.SubElement(proto_element, 'documentation-url').text = info.documentation_url
        if info.description != '':
            description_element = ET.SubElement(proto_element, 'description').text = info.description
        if info.slot_type is not None:
            slot_element = ET.SubElement(proto_element, 'slot-type').text = info.slot_type
        if info.tags:
            tags_element = ET.SubElement(proto_element, 'tags').text = ','.join(info.tags)
        if info.needs_robot_ancestor:
            robot_ancestor_element = ET.SubElement(proto_element, 'needs-robot-ancestor').text = 'true'

    # beautify xml
    tree = ET.ElementTree(root)
    xml_string = xml.dom.minidom.parseString(ET.tostring(root)).toprettyxml(encoding='utf-8')

    # save to file
    filename = f'{WEBOTS_HOME}/resources/proto-list.xml'
    if (os.path.exists(filename)):
        os.remove(filename)

    with open(filename, 'wb') as file:
        file.write(xml_string)

    # regenerate hashfile, either it didn't exist to begin with or was incomplete
    if os.path.exists(hashfile):
        os.remove(hashfile)

    with open(hashfile, 'w') as file:
        for key, info in protos.items():
            hash = hashlib.sha1(info.contents.encode('utf-8')).hexdigest()
            file.write(f'{key} {hash}\n')
