#!/usr/bin/env python3

# Copyright 1996-2021 Cyberbotics Ltd.
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

"""Convert R2020b world file from the NUE to the ENU coordinate system."""

import math
import sys

from transforms3d import quaternions

from webots_parser import WebotsParser

coordinateSystem = 'ENU'


def rotation(value, r):
    q0 = quaternions.axangle2quat([float(value[0]), float(value[1]), float(value[2])], float(value[3]))
    q1 = quaternions.axangle2quat([r[0], r[1], r[2]], r[3])
    qr = quaternions.qmult(q1, q0)
    v, theta = quaternions.quat2axangle(qr)
    return [WebotsParser.str(v[0]), WebotsParser.str(v[1]), WebotsParser.str(v[2]), WebotsParser.str(theta)]


def createNewTransform():
    if coordinateSystem == 'ENU':
        return {'fields': [{'name': 'Geometrics_conversion', 'value': 'Geometrics_conversion',
                                    'type': 'SFString'}, {'name': 'rotation', 'value': ['1', '0', '0', '1.57'],
                                                          'type': 'SFRotation'},
                           {'name': 'children', 'type': 'MFNode', 'value': []}],
                'type': 'node', 'name': 'Transform'}
    else:
        return {'fields': [{'name': 'Geometrics_conversion', 'value': 'Geometrics_conversion',
                                    'type': 'SFString'}, {'name': 'rotation', 'value': ['1', '0', '0', '-1.57'],
                                                          'type': 'SFRotation'},
                           {'name': 'children', 'type': 'MFNode', 'value': []}],
                'type': 'node', 'name': 'Transform'}


def convert_children(node, parent):
    if 'USE' in node:
        return
    if node['name'] in 'Shape':
        for field in node['fields']:
            if field['name'] in ['geometry'] and field['value']['name'] in ['Cylinder', 'Cone', 'Capsule', 'ElevationGrid',
                                                                            'Plane']:
                newTransform = createNewTransform()
                for param in newTransform['fields']:
                    if param['name'] in 'children':
                        param['value'] = [node]
                        parent.remove(node)

                parent.append(newTransform)
    #  Case of boundingObject
    elif node['name'] in ['Cylinder', 'Capsule', 'ElevationGrid', 'Plane']:
        newTransform = createNewTransform()
        for param in newTransform['fields']:
            if param['name'] in 'children':
                param['value'] = [node]

        for field in parent['fields']:
            if field['name'] in 'boundingObject':
                field['value'] = newTransform
    else:
        for field in node['fields']:
            if field['name'] in 'Geometrics_conversion':
                break
            elif field['name'] in 'children':
                for child in field['value']:
                    convert_children(child, field['value'])
            elif field['name'] in 'endPoint':
                convert_children(field['value'], field['value'])
            elif field['name'] in 'boundingObject':
                convert_children(field['value'], node)


def cleanTransform(node):
    for field in node['fields']:
        if field['name'] in 'Geometrics_conversion':
            node['fields'].remove(field)
            break
        elif field['name'] in 'children':
            for child in field['value']:
                cleanTransform(child)
        elif field['name'] in ['endPoint', 'boundingObject']:
            cleanTransform(field['value'])


def squashUniqueTransform(node):
    if node['name'] in 'Transform':
        for field in node['fields']:
            if field['name'] in 'children':
                if len(field['value']) == 1 and field['value'][0]['name'] in 'Transform':
                    childT = field['value'][0]
                    for fieldC in childT['fields']:
                        if fieldC['name'] == 'Geometrics_conversion':
                            mergeTransform(node, childT)

    for field in node['fields']:
        if field['name'] in 'children':
            for child in field['value']:
                squashUniqueTransform(child)
        elif field['name'] in ['endPoint']:
            squashUniqueTransform(field['value'])


def mergeTransform(parent, child):
    childRotation = None
    isChildRotation = False
    childShape = None
    isChildShape = False
    for childField in child['fields']:
        if childField['name'] in 'rotation':
            childRotation = childField['value']
            isChildRotation = True
        if childField['name'] in 'children':
            if childField['value'][0]['name'] in 'Shape':
                childShape = childField['value'][0]
                isChildShape = True

    if isChildRotation and isChildShape:
        isParentRotation = False
        for parentField in parent['fields']:
            if parentField['name'] in 'rotation':
                parentField['value'] = rotation(parentField['value'], childRotation)
                isParentRotation = True
            if parentField['name'] in 'children':
                parentField['value'] = [childShape]
        if not isParentRotation:
            parent['fields'].append({'name': 'rotation',
                                     'value': childRotation,
                                     'type': 'SFRotation'})


def convert_to_enu(filename):
    world = WebotsParser()
    world.load(filename)
    global coordinateSystem
    print("Add transforms")
    for node in world.content['root']:
        if node['name'] == 'WorldInfo':
            for field in node['fields']:
                if field['name'] in 'coordinateSystem':
                    coordinateSystem = field['value']
        else:
            convert_children(node, world.content['root'])

    print("Merge transforms")
    for node in world.content['root']:
        squashUniqueTransform(node)
        cleanTransform(node)

    world.save(filename)


if __name__ == "__main__":
    # execute only if run as a script
    # for filename in sys.argv:
    #     if not filename.endswith('.wbt'):
    #         continue
    filename = "tests/api/worlds/accelerometer.wbt"
    print(filename)
    convert_to_enu(filename)
