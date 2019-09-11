# Copyright 1996-2019 Cyberbotics Ltd.
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

"""Vec3 class container."""

import math


class Vec3(object):
    """Abstraction of mathematical 3D vector."""

    __slots__ = ['x', 'y', 'z']

    def __init__(self, x=0.0, y=0.0, z=0.0):
        """Constructor: set the vector coordinate."""
        self.x = x
        self.y = y
        self.z = z

    def __repr__(self):
        """To String conversion."""
        return 'vec3({}, {})'.format(self.x, self.y, self.z)

    def __eq__(self, other):
        """Equality operator."""
        if isinstance(other, Vec3):
            return self.x == other.x and self.y == other.y and self.z == other.z
        return False

    def __ne__(self, other):
        """Equality operator."""
        return not self == other

    def __add__(self, other):
        """Add the vector with another."""
        return Vec3(self.x + other.x, self.y + other.y, self.z + other.z)

    def __sub__(self, other):
        """Substract the vector to another."""
        return Vec3(self.x - other.x, self.y - other.y, self.z - other.z)

    def __mul__(self, other):
        """Multiply the vector with another."""
        if isinstance(other, Vec3):
            #  Dot product
            return self.x * other.x + self.y * other.y + self.z * other.z
        elif isinstance(other, float):
            #  Scalar product
            return Vec3(self.x * other, self.y * other, self.z * other)
        else:
            raise TypeError()

    def __truediv__(self, other):
        """Divide the vector to another."""
        if isinstance(other, Vec3):
            #  Dot product
            return self.x / other.x + self.y / other.y + self.z / other.z
        elif isinstance(other, float):
            #  Scalar product
            return Vec3(self.x / other, self.y / other, self.z / other)
        else:
            raise TypeError()

    def __floordiv__(self, other):
        """Divide the vector to another."""
        return self.__truediv__(other)

    def __div__(self, other):
        """Divide the vector to another."""
        return self.__truediv__(other)

    def norm(self):
        """Compute the vector norm."""
        return math.sqrt(self.x * self.x + self.y * self.y + self.z * self.z)

    def normalize(self):
        """Normalize the vector."""
        return self / self.norm()

    def cross(self, other):
        """Cross product."""
        return Vec3(
            self.y * other.z - self.z * other.y,
            self.z * other.x - self.x * other.z,
            self.x * other.y - self.y * other.x
        )
