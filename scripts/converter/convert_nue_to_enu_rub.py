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

'''
**Presentation**
        This script intends to help you convert protos and worlds in RUB.
        However, since the rotation depends on the proto itself,
        it could be needed to rotate some part manually.
        Also, we advice you to check the changes in a file comparator.

**Dependencies**
  `pip3 install numpy transforms3d`

**Usage**
  To select your file, your can add one or multiple filename.s:
      - by changing the `filename_list` variable
      - by adding an argument in the terminal
      - by changing the variable `foldername` to convert all the `.wbt` and `.proto` of a folder
   You can choose the mode:
      - all: convert and clean the `.wbt` or `.proto` file in RUB.
      - clean: delete the useless lines (rotation with angle 0 or useless precision) of protos or worlds.
        It also include the possibility to round the values.
      - specific: convert a specific field, see an example line 157  of the script.

***file structure***
    The `centerOfMass` and the geometry `IndexedFaceSet` need to have this specific structure (only carriage returns matter):
```
centerOfMass [
    Vx Vy Vz
]
```
```
geometry IndexedFaceSet {
  coord Coordinate {
    point [
      ....
    ]
  }
}
```

***To do manually***
        After the conversion, a verbose indicates you if the conversion is incomplete or not.
        If Yes, it indicates you which part(s) of the file(s) you will have to change manually.
            * if `JointParameters` have no axis, you need to change it manually
            in RUB, for sliderJoint add `axis 1 0 0 `; for HingeJoint add: `axis 0 -1 0`
            CTRL-F on HingeJointParameters
            * You may have to change the sensors manually.
            * For inertiaMatrix, RUB to FLU is:
```
            [I11, I22, I33]  =>  [I33, I11, I22]
            [I12, I13, I23]      [I13, -I23, -I12]
```
            * To convert elevationGrid you normally need to:
              - replace zDimension et zSpacing par y
              - translate it on y by -(yDimension -1) * ySpacing
              - inverse the lines of heights with convert.py

**Conversion process**
    Here is a list of the conversion process:
        - replace `R2021b` by `R2021c`
        - remove the `coordinateSystem ENU` line
        - convert the orientation of the viewpoint: [Ox, Oy, Oz, Oa] --> [Ox, Oz, Oy, Oa]
        - convert the position of the viewpoint: [Px, Py, Pz] --> [-Pz, -Px, Py]
        - convert the vector of the keyword 'translation', 'axis', 'anchor',
          'location', 'direction': [Vx, Vy, Vz] --> [-Vz, -Vx, Vy]
        - convert the vector of the keyword 'rotation': [Rx, Ry, Rz, Ra] --> [-Rz, -Rx, Ry, Ra]
        - convert the vector of the keyword 'size', 'frameSize', 'stepSize': [Vx, Vy, Vz] --> [Vz, Vx, Vy]
        - convert the line after the keyword 'centerOfMass' (see 'file structure' above): [Vx, Vy, Vz] --> [-Vz, -Vx, Vy]
        - if it finds the keyword 'coord', skip one line (the line 'point [', see 'file structure' above)
        and convert all the geometry points until it reaches ']' : [Vx, Vy, Vz] --> [-Vz, -Vx, Vy]

'''

import sys
import os
import re
import fileinput
import numpy as np
import transforms3d


def is_number(string):
    try:
        float(string)
        return True
    except ValueError:
        return False


def clean_vector(vector, decimals=4, zero_one_decimals=None, round_option=False):
    if vector is None:
        return None
    else:
        if zero_one_decimals is None:
            # Zero and one are special cases and typically it is fine to be more agressive when rounding
            zero_one_decimals = int(0.7 * decimals)
        for id, value in enumerate(vector):
            if abs(value) < 1 / (10**zero_one_decimals):
                vector[id] = 0
            elif abs(value - 1) < 1 / (10**zero_one_decimals):
                vector[id] = 1
            elif abs(value + 1) < 1 / (10**zero_one_decimals):
                vector[id] = -1
            elif round_option:
                vector[id] = round(vector[id], decimals)
            else:
                vector[id] = vector[id]

            if id == 3 and vector[id] == 0:
                vector = None

        return vector


def add_space(string):
    space = ''
    for c in string:
        if c == ' ':
            space += ' '
        else:
            return space


def convert_translation(translation):
    ROTATION = [np.pi / 2, -np.pi / 2, 0]
    ROTATION_MATRIX = transforms3d.euler.euler2mat(ROTATION[0], ROTATION[1], ROTATION[2], 'rxyz')

    translation = [float(value) for value in translation]
    return (ROTATION_MATRIX @ np.array(translation)).flatten()


def convert_mesh(geometry_points):
    for i in range(0, len(geometry_points), 3):
        new_point = convert_translation(geometry_points[i:i + 3])
        new_point_cleaned = clean_vector(new_point, decimals=6, round_option=True)
        geometry_points[i:i + 3] = new_point_cleaned
    return geometry_points


def convert_nue_to_enu_world(filename, mode='all'):
    error_verbose = ''  # print the fields that need to be done manually
    clean_verbose = ''  # print the lines where the rotation/translation have been cleaned
    next_line_is_coord = -1
    next_line_is_com = -1
    HingeJoint_count = 0
    for line in fileinput.input(filename, inplace=True):

        # extract the type of field and its associated vector of line
        type = [x for x in re.compile('\n|,| ').split(line) if x.isalpha()]
        vector = [float(x) for x in re.compile('\n|,| ').split(line) if is_number(x)]
        if type:
            type = type[0]
        if ('ElevationGrid' in line and '{' in line):
            type = 'ElevationGrid'
        write_status = True

        if mode == 'specific':  # to change only a specific field
            if type in ['scale'] and len(vector) == 2:  # for example, all the fields scale
                vector = [vector[1], vector[0]]
            else:
                write_status = False
        elif mode == 'clean':  # to clean the rotation and translation (aims to avoid useless precision and lines)
            if type not in ['rotation', 'translation']:
                write_status = False
        elif mode == 'all':
            if 'R2021b' in line:
                line = '#VRML_SIM R2021c utf8 \r\n'
            elif 'R2021c' in line:
                error_verbose += 'Warning: The version of the file is already 2021c. '
            if type in ['coordinateSystem']:  # remove the 'coordinateSystem ENU'
                vector = None
            elif type in ['orientation'] and len(vector) == 4:
                vector = [vector[0], vector[2], vector[1], vector[3]]  # orientation Ox Oy Oz Oa --> Ox Oz Oy Oa
            elif type in ['position'] and len(vector) == 3:
                vector = [-vector[2], -vector[0], vector[1]]  # position Px Py Pz --> -Pz -Px Py
            elif type in ['translation', 'axis', 'anchor', 'location', 'direction'] and len(vector) == 3:
                vector = [-vector[2], -vector[0], vector[1]]  # RUB
                if type in ['axis']:
                    HingeJoint_count -= 1
            elif type in ['rotation'] and len(vector) == 4:
                vector = [-vector[2], -vector[0], vector[1], vector[3]]  # RUB
            elif type in ['size', 'frameSize', 'stepSize'] and len(vector) == 3:
                vector = [vector[2], vector[0], vector[1]]
            else:
                write_status = False  # else we do not change the line

                # verbose to print, fields to change manually
                if type in ['inertiaMatrix', 'DistanceSensor', 'LightSensor', 'ElevationGrid']:
                    error_verbose += 'line ' + str(fileinput.lineno()) + ': ' + type + '  ;  '
                elif type in ['jointParameters', 'jointParameters2']:
                    HingeJoint_count += 1

                if type in ['centerOfMass']:
                    next_line_is_com = 1
                elif next_line_is_com == 1:  # we skip one line if type was 'centerOfMass'
                    write_status = True
                    if len(vector) == 3:
                        vector = [-vector[2], -vector[0], vector[1]]

                if type in ['coord']:
                    next_line_is_coord = 2
                elif next_line_is_coord >= 2:  # we skip 2 lines if type was 'coord'
                    next_line_is_coord -= 1
                elif next_line_is_coord == 1:
                    if ']' in line:  # we stop when we reach the end of the node 'point' of 'coord'
                        next_line_is_coord = -1
                    else:  # else we convert the line
                        vector = convert_mesh(vector)
                        vector_str = [str(i) for i in vector]
                        line = add_space(line) + ' '.join(vector_str) + '\r\n'

        if write_status:
            # we clean the vector
            vector_cleaned = clean_vector(vector, decimals=6)
            if vector_cleaned != vector:
                clean_verbose += (' line {},'.format(fileinput.lineno()))
            # we replace the vector by the new one.
            if vector_cleaned is None:
                line = ''
            else:
                vector_str = [str(i) for i in vector_cleaned]
                if next_line_is_com == 1:
                    line = add_space(line) + ' '.join(vector_str) + '\r\n'
                    next_line_is_com = -1
                elif isinstance(type, str):
                    line = add_space(line) + type + ' ' + ' '.join(vector_str) + '\r\n'

        print('{}'.format(line), end='')  # write 'line' in the file

    if HingeJoint_count:
        error_verbose += '{} JointParameters with missing axis field'.format(HingeJoint_count)
    return error_verbose, clean_verbose


if __name__ == '__main__':

    mode = 'all'  # specific, clean or all
    # possibility to use an argv, a list or a folder
    filename_list = []  # example: 'projects/robots/robotcub/icub/worlds/icub_stand.wbt', change it by your .wbt or.proto
    foldername = ''  # example: 'projects/robots/parallax/boebot/protos/', change it by your .wbt or.proto folder

    if len(sys.argv) == 2:
        filename_list = [str(sys.argv[1])]
    elif not filename_list:
        if not foldername:
            raise ValueError(
                "filename_list empty, add an argument with the path or change filename_list or foldername variables")
        filename_full_list = os.listdir(foldername)
        for filename in filename_full_list:
            if '.wbt' in filename or '.proto' in filename:
                filename_list.append(foldername + filename)
    for filename in filename_list:
        error_verbose, clean_verbose = convert_nue_to_enu_world(filename, mode=mode)
        if error_verbose:
            print('Conversion of \033[33m{}\033[m successfull but incomplete. \r\n■ Need to convert manually: {}'.format(
                filename, error_verbose))
        else:
            if mode == 'clean':
                print('clean of {} ✅'.format(filename))
            else:
                print('Conversion of {} ✅'.format(filename))
        if clean_verbose:
            print('Cleaned lines:', clean_verbose[:-1])
    print('Achieved successfully!')
