#!/usr/bin/env python3
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

"""This module gather the OSM parser classes."""

from osm_objects import OSMCoord
from osm_objects import OSMMultipolygon
from osm_objects import OSMNode

from webots_objects.area import Area
from webots_objects.barrier import Barrier
from webots_objects.building import Building
from webots_objects.parking_lines import ParkingLines
from webots_objects.river import River
from webots_objects.road import Road
from webots_objects.tree import Tree

import sys
try:
    from lxml import etree
except Exception:
    sys.exit("Error: lxml python module not installed. You can install it using pip: 'pip install lxml'")


class Parser(object):
    """Parser of the OSM file."""

    wayRefList = {}
    relations = []

    def parse_file(self, file, disableMultipolygonBuildings=False):
        """Parse the OSM file."""
        tree = etree.parse(file)
        root = tree.getroot()

        if not etree.iselement(root):
            sys.exit("Error while parsing '" + file + "' file.\n")
        for node in root.findall('node'):
            self.parse_node(node)
        for way in root.findall('way'):
            self.parse_way(way)
        for relation in root.findall('relation'):
            self.parse_relation(relation)
        self.process_relations(disableMultipolygonBuildings)

        # Define the most likely country, based on the occurencies of the "addr:country" tags
        # (which can occur at different locations).
        self.country = None
        try:
            countryTags = tree.xpath("//tag[@k='addr:country']")
            self.country = sorted(countryTags, key=lambda x: countryTags.count(x.attrib['v']))[-1].attrib['v']
        except Exception:
            pass
        if self.country is None:
            print('Warning: Failed to determine the country.')

    def get_tags(self, element):
        """Return a dictionnary of tags belonging to this element."""
        tags = {}
        for tag in element.findall('tag'):
            tags[tag.attrib['k']] = tag.attrib['v']
        return tags

    def parse_node(self, node):
        """Parse a node element and create the appropriate OSMNode, OSMCoord and if needed Tree object."""
        lat = node.attrib['lat']
        lon = node.attrib['lon']
        osmId = node.attrib['id']
        tags = self.get_tags(node)
        OSMNode.add(osmId, lon, lat, tags)
        OSMCoord.add(osmId, lon, lat)
        if 'natural' in tags:
            tree = Tree()
            if 'height' in tags:
                tree.height = float(tags['height'])
            if 'leaf_type' in tags:
                tree.leafType = tags['leaf_type']
            if 'diameter_crown' in tags:
                tree.radius = tags['diameter_crown'] / 2.0
            tree.coord = OSMCoord.coordDictionnary[osmId]
            Tree.list.append(tree)

    def parse_way(self, way):
        """Parse a way element and create the corresponding object."""
        osmId = way.attrib['id']
        tags = self.get_tags(way)
        refs = []
        for ref in way.findall('nd'):
            refs.append(ref.attrib['ref'])
        self.wayRefList[osmId] = refs  # we need to store them because the can then be usefull when parsin 'relations'

        # dont take into acount underground structure
        if float(tags.get('layer', '0')) < 0:
            return

        if 'building' in tags or 'building:part' in tags:
            Building.add_to_list(osmId, tags, refs)
        elif 'highway' in tags:
            Road.add_to_list(osmId, tags, refs)
        elif tags.get('waterway') == 'river' or tags.get('waterway') == 'stream':
            River.add_to_list(osmId, tags, refs)
        elif tags.get('barrier') == 'fence' or tags.get('barrier') == 'wall':
            Barrier.add_to_list(osmId, tags, refs)
        elif 'natural' in tags:
            Area.add_to_list(osmId, tags['natural'], tags, refs)
        elif 'landuse' in tags:
            Area.add_to_list(osmId, tags['landuse'], tags, refs)
        elif 'waterway' in tags:
            Area.add_to_list(osmId, tags['waterway'], tags, refs)
        elif 'amenity' in tags and tags['amenity'] == 'parking':
            Area.add_to_list(osmId, 'parking', tags, refs)
        elif 'name' in tags and tags['name'] == 'parking line':
            ParkingLines.add_to_list(osmId, tags, refs)

    def parse_relation(self, relation):
        """Parse a relation element and store it in self.relations."""
        currentRelation = {
            'osmId': relation.attrib['id'],
            'tags': self.get_tags(relation),
            'members': []
        }
        for member in relation.findall('member'):
            currentRelation['members'].append({
                'reference': member.attrib['ref'],
                'type': member.attrib['type'],
                'role': member.attrib['role']
            })
        self.relations.append(currentRelation)

    def process_relations(self, disableMultipolygonBuildings):
        """Process all the multipolygons."""
        for relation in self.relations:  # reference, type, role
            OSMMultipolygon.add(relation['osmId'], relation['tags'], relation['members'], self.wayRefList)
        OSMMultipolygon.process(disableMultipolygonBuildings)
        if not disableMultipolygonBuildings:
            for multipolygon in OSMMultipolygon.multipolygonList:
                # create a building from the multipolygon
                if 'building' in multipolygon.tags and not OSMMultipolygon.disableMultipolygonBuildings:
                    Building.add_to_list(multipolygon.OSMID, multipolygon.tags, multipolygon.ref)
                # create an area from the multipolygon
                elif 'natural' in multipolygon.tags:
                    Area.add_to_list(multipolygon.OSMID, multipolygon.tags['natural'], multipolygon.tags, multipolygon.ref)
                elif 'landuse' in multipolygon.tags:
                    Area.add_to_list(multipolygon.OSMID, multipolygon.tags['landuse'], multipolygon.tags, multipolygon.ref)
                elif 'amenity' in multipolygon.tags and multipolygon.tags['amenity'] == 'parking':
                    Area.add_to_list(multipolygon.OSMID, 'parking', multipolygon.tags, multipolygon.ref)
