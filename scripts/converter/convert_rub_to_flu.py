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

"""Convert Webots PROTO file from the RUB (x-Right, y-Up, z-Back) to FLU (x-Forward, y-Left, z-Up)."""

import sys
import numpy as np
import transforms3d
from webots_parser import WebotsParser


ROTATION_RUB_TO_FLU = transforms3d.axangles.axangle2mat(
    [-0.5773516025189619, 0.5773476025217157, 0.5773516025189619], -2.094405307179586)


def find(function, elements, default=None):
    return next((x for x in elements if function(x)), default)


def get_field(node, field_name, default=None):
    return find(lambda x: x['name'] == field_name, node['fields'], default)


def get_vector3(node, name='translation'):
    vector3_field = get_field(node, name)
    if not vector3_field:
        return [0, 0, 0]
    return [float(value) for value in vector3_field['value']]


def get_rotation(node):
    rotation_field = get_field(node, 'rotation')
    if not rotation_field:
        return [1, 0, 0, 0]
    return [float(value) for value in rotation_field['value']]


def set_rotation(node, value):
    rotation_field = get_field(node, 'rotation')
    if not rotation_field:
        node['fields'].append({'name': 'rotation',
                               'value': value,
                               'type': 'SFRotation'})
        return
    rotation_field['value'] = value


def set_vector3(node, value, name='translation'):
    vector3_field = get_field(node, name)
    if not vector3_field:
        node['fields'].append({'name': name,
                               'value': value,
                               'type': 'SFVec3f'})
        return
    vector3_field['value'] = value


def convert_pose(rotation_angle_axis, translation):
    assert len(rotation_angle_axis) == 4, f'Rotation {rotation_angle_axis} is not the correct length'
    assert len(translation) == 3, f'Translation {translation} is not the correct length'

    # Convert
    rotation = transforms3d.axangles.axangle2mat(rotation_angle_axis[:3], rotation_angle_axis[3])
    new_rotation = ROTATION_RUB_TO_FLU @ rotation
    new_rotation_axis, new_rotation_angle = transforms3d.axangles.mat2axangle(new_rotation)
    new_translation = ROTATION_RUB_TO_FLU @ np.array(translation)

    # Convert to string array
    new_rotation_str = [f'{round(v, 4):.4}' for v in new_rotation_axis.tolist() + [new_rotation_angle]]
    new_translation_str = [f'{round(v, 3):.3}' for v in new_translation]

    return new_rotation_str, new_translation_str


def convert_nodes(nodes):
    for node in nodes:
        if 'Hinge' in node['name']:
            # TODO: This part needs more love :) We need to convert endpoints as well.
            joint_parameters_node = get_field(node, 'jointParameters')['value']

            anchor = get_vector3(joint_parameters_node, name='anchor')
            new_anchor = ROTATION_RUB_TO_FLU @ np.array(anchor)
            new_anchor_str = [f'{round(v, 5):.5}' for v in new_anchor]
            set_vector3(joint_parameters_node, new_anchor_str, name='anchor')

            axis = get_vector3(joint_parameters_node, name='axis')
            new_axis = ROTATION_RUB_TO_FLU @ np.array(axis)
            new_axis_str = [f'{round(v, 5):.5}' for v in new_axis]
            set_vector3(joint_parameters_node, new_axis_str, name='axis')

        elif node['name'] == 'Group':
            children = get_field(node, 'children')
            if children and children['type'] != 'IS':
                convert_nodes(children['value'])
        elif node['name'] == 'Shape':
            pass
        else:
            translation = get_vector3(node)
            rotation = get_rotation(node)
            new_rotation, new_translaton = convert_pose(rotation, translation)
            set_rotation(node, new_rotation)
            set_vector3(node, new_translaton)


def main():
    proto = WebotsParser()
    proto.load(sys.argv[1])

    # Find the Robot's children field
    # root > PROTO (`[0]['root']`) > Robot (`[0]`) > fields (`['fields']`)
    robot_node = proto.content['root'][0]['root'][0]

    # Convert robot's children
    robot_children = get_field(robot_node, 'children')['value']
    convert_nodes(robot_children)

    # Convert bounding objects
    bounding_object = get_field(robot_node, 'boundingObject')['value']
    if bounding_object:
        convert_nodes([bounding_object])

    proto.save(sys.argv[1])


if __name__ == '__main__':
    main()
