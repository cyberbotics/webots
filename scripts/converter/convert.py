# Converts rotation and translation VRML fields.
# It takes the clipboard content, applies transforms, and pastes the transformed VRML to the clipboard.
#
# GNOME SHORTCUT
# In GNOME, you can set to run this script as a keyboard shortcut command.
# Go to `Settings > Keyboard Shortcuts > +`

import clipboard
import pyperclip
import numpy as np
import transforms3d
import re


# ROTATION = [0.577351, -0.577351, 0.577351, 2.09439]
# ROTATION = [0.577351, -0.577351, -0.577351, -2.09439]
ROTATION = [-0.5773516025189619, 0.5773476025217157, 0.5773516025189619, -2.094405307179586]    # From RUB to FLU
ROTATION = [0, -0.707, 0.707, 3.140466]  # Plane, from RUB to FLU
# ROTATION = [1, 0, 0, -np.pi/2]  # Plane, from RUB to FLU

# ROTATION = [0, 1, 0, -np.pi/2]  # Cylinder after conversion from RUB

ROTATION_MATRIX = transforms3d.axangles.axangle2mat(ROTATION[:3], ROTATION[3])


ROTATION = [-np.pi/2, np.pi, 0]
ROTATION_MATRIX = transforms3d.euler.euler2mat(ROTATION[0], ROTATION[1], ROTATION[2], 'rxyz')

ROTATION_MATRIX = transforms3d.axangles.axangle2mat(
        [-0.5773516025189619, 0.5773476025217157, 0.5773516025189619], -2.094405307179586)
ROTATION_MATRIX = transforms3d.axangles.axangle2mat(
                [0, 0, 1], 1.57079632679)


def vector_to_string_array(vector, decimals, zero_one_decimals=None):
    if zero_one_decimals is None:
        # Zero and one are special cases and typically it is fine to be more agressive when rounding
        zero_one_decimals = int(0.7 * decimals)
    new_str = []
    for value in vector:
        if abs(value) < 1 / (10**zero_one_decimals):
            new_str.append('0')
        elif abs(value - 1) < 1 / (10**zero_one_decimals):
            new_str.append('1')
        else:
            new_str.append(str(round(value, decimals)))
    return new_str


def convert_translation(translation):
    translation = [float(value) for value in translation]
    return (ROTATION_MATRIX @ np.array(translation)).flatten()


def convert_mesh(geometry_points):
    for i in range(0, len(geometry_points), 3):
        new_point = convert_translation(geometry_points[i:i+3])
        geometry_points[i:i+3] = vector_to_string_array(new_point, 7)
    return ' '.join(geometry_points)


def convert_orientation(rotation_angle_axis):
    rotation_angle_axis = [float(value) for value in rotation_angle_axis]
    orientation = transforms3d.axangles.axangle2mat(rotation_angle_axis[:3], rotation_angle_axis[3])
    new_rotation = ROTATION_MATRIX @ orientation
    new_rotation_axis, new_rotation_angle = transforms3d.axangles.mat2axangle(new_rotation)
    return ' '.join(vector_to_string_array(list(new_rotation_axis) + list([new_rotation_angle]), 6))


def is_number(string):
    try:
        float(string)
        return True
    except ValueError:
        return False


def main():
    text = clipboard.paste()
    vector = [x for x in re.compile('\n|,| ').split(text) if is_number(x)]
    out = 'ERROR'

    if len(vector) == 0:
        new_rotation_axis, new_rotation_angle = transforms3d.axangles.mat2axangle(ROTATION_MATRIX)
        out = ' '.join(vector_to_string_array(list(new_rotation_axis) + list([new_rotation_angle]), 6))
    elif len(vector) == 3:
        out = convert_translation(vector)
        out = ' '.join(vector_to_string_array(out, 4))
    elif len(vector) == 4:
        out = convert_orientation(vector)
    else:
        out = convert_mesh(vector)

    print(out)
    clipboard.copy(out)


if __name__ == '__main__':
    main()
