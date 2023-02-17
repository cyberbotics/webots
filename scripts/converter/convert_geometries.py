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

"""Convert R2020b world file from the NUE to the ENU coordinate system."""

from transforms3d import quaternions

from webots_parser import WebotsParser

coordinateSystem = 'ENU'


def rotation(value, r):
    # if value == ['0', '0', '1', '1.57'] and r == ['1', '0', '0', '-1.57079632679']:
    #     return [WebotsParser.str(0), WebotsParser.str(1), WebotsParser.str(0), WebotsParser.str(-1.57079632679)]

    q0 = quaternions.axangle2quat([float(value[0]), float(value[1]), float(value[2])], float(value[3]))
    q1 = quaternions.axangle2quat([float(r[0]), float(r[1]), float(r[2])], float(r[3]))
    qr = quaternions.qmult(q1, q0)
    v, theta = quaternions.quat2axangle(qr)
    return [WebotsParser.str(v[0]), WebotsParser.str(v[1]), WebotsParser.str(v[2]), WebotsParser.str(theta)]


def createNewTransform(coordinateSystem):
    if coordinateSystem == 'ENU':
        return {'fields': [{'name': 'Geometrics_conversion', 'value': 'Geometrics_conversion',
                                    'type': 'SFString'}, {'name': 'rotation', 'value': ['1', '0', '0', '1.57079632679'],
                                                          'type': 'SFRotation'},
                           {'name': 'children', 'type': 'MFNode', 'value': []}],
                'type': 'node', 'name': 'Transform'}
    else:
        return {'fields': [{'name': 'Geometrics_conversion', 'value': 'Geometrics_conversion',
                                    'type': 'SFString'}, {'name': 'rotation', 'value': ['1', '0', '0', '-1.57079632679'],
                                                          'type': 'SFRotation'},
                           {'name': 'children', 'type': 'MFNode', 'value': []}],
                'type': 'node', 'name': 'Transform'}


def convert_children(node, parent, coordinateSystem):
    if 'USE' in node:
        return
    if node['name'] in 'Shape':
        for field in node['fields']:
            if field['name'] in ['geometry'] and 'USE' not in field['value'] and field['value']['name'] in ['Cylinder', 'Cone',
                                                                                                            'Capsule',
                                                                                                            'ElevationGrid',
                                                                                                            'Plane']:
                isDef = False
                defName = None
                position = -1
                for index in range(0, len(parent)):
                    if parent[index] == node:
                        position = index
                #  We need to transfer the def of the geometry to the transform
                if 'DEF' in field['value']:
                    defName = field['value']['DEF']
                    isDef = True
                    field['value'].pop('DEF')
                #  We need to transfer the def of the shape to the transform
                if 'DEF' in node:
                    defName = node['DEF']
                    isDef = True
                    node.pop('DEF')

                newTransform = createNewTransform(coordinateSystem)
                for param in newTransform['fields']:
                    if param['name'] in 'children':
                        param['value'] = [node]

                if isDef:
                    newTransform['DEF'] = defName

                parent[position] = newTransform
    # Case of boundingObject
    elif node['name'] in ['Cylinder', 'Capsule', 'ElevationGrid', 'Plane']:
        newTransform = createNewTransform(coordinateSystem)
        for param in newTransform['fields']:
            if param['name'] in 'children':
                param['value'] = [node]
        # Case where this is a geometry directly inserted in a boundingObject
        if 'fields' in parent:
            for field in parent['fields']:
                if field['name'] in 'boundingObject':
                    field['value'] = newTransform
        # Case where this is a geometry in a transform
        else:
            parent.remove(node)
            parent.append(newTransform)

    for field in node['fields']:
        if field['name'] in 'Geometrics_conversion':
            break
        elif field['name'] in 'children':
            for child in field['value']:
                convert_children(child, field['value'], coordinateSystem)
        elif field['name'] in 'endPoint':
            convert_children(field['value'], field['value'], coordinateSystem)
        elif field['name'] in 'boundingObject':
            convert_children(field['value'], node, coordinateSystem)


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
    if 'USE' in node:
        return
    if node['name'] in ['Transform']:
        for field in node['fields']:
            if field['name'] in 'children':
                if len(field['value']) == 1 and field['value'][0]['name'] in 'Transform' and 'DEF' not in field['value'][0]:
                    childT = field['value'][0]
                    for fieldC in childT['fields']:
                        if fieldC['name'] == 'Geometrics_conversion':
                            mergeTransform(node, childT)

    for field in node['fields']:
        if field['name'] in 'children':
            for child in field['value']:
                squashUniqueTransform(child)
        elif field['name'] in ['endPoint', 'boundingObject']:
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
            if childField['value'][0]['name'] in ['Shape', 'Cylinder', 'Capsule', 'ElevationGrid', 'Plane']:
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
    print("Add transforms")
    for node in world.content['root']:
        if node['name'] == 'WorldInfo':
            for field in node['fields']:
                if field['name'] in 'coordinateSystem':
                    coordinateSystem = field['value']
        else:
            convert_children(node, world.content['root'], coordinateSystem)

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
    filename = "tests/api/worlds/range_finder.wbt"
    print(filename)
    convert_to_enu(filename)
