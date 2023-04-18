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

'''
**Presentation**
    Convert .forest to flu to meet the forest.proto R2022a version.

**Usage**
    You can either add an argument or change the filename_list or foldername variables.

'''

import sys
import os
import re
import fileinput


def is_number(string):
    try:
        float(string)
        return True
    except ValueError:
        return False


def convert_forest(filename):

    for line in fileinput.input(filename, inplace=True):
        vector = [float(x) for x in re.compile('\n|,| ').split(line) if is_number(x)]
        vector = [vector[0], vector[2], vector[1]]

        vector_str = [str(i) for i in vector]
        line = ','.join(vector_str) + '\n'
        print('{}'.format(line), end='')  # write 'line' in the file


if __name__ == '__main__':

    # example: 'projects/vehicles/worlds/forest/village/-1986.forest'
    filename_list = []

    # example: 'projects/vehicles/worlds/forest/village_winter/', change it by your forest folder
    foldername = ''

    if len(sys.argv) == 2:
        filename_list = [str(sys.argv[1])]
    elif not filename_list:
        if not foldername:
            raise ValueError(
                "filename_list empty, add an argument with the path or change filename_list or foldername variables")
        filename_full_list = os.listdir(foldername)
        for filename in filename_full_list:
            if '.forest' in filename:
                filename_list.append(foldername + filename)
    for filename in filename_list:
        convert_forest(filename)
        print('Conversion of {} ✅'.format(filename))
