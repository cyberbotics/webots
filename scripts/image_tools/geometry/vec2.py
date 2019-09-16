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

"""Vec2 class container."""

import math


class Vec2(object):
    """Abstraction of mathematical 2D vector."""

    __slots__ = ['x', 'y']

    def __init__(self, x=0.0, y=0.0):
        """Constructor: set the vector coordinate."""
        self.x = x
        self.y = y

    def __repr__(self):
        """To String conversion."""
        return 'vec2({}, {})'.format(self.x, self.y)

    def __eq__(self, other):
        """Equality operator."""
        if isinstance(other, Vec2):
            return self.x == other.x and self.y == other.y
        return False

    def __ne__(self, other):
        """Equality operator."""
        return not self == other

    def __add__(self, other):
        """Add the vector with another."""
        return Vec2(self.x + other.x, self.y + other.y)

    def __sub__(self, other):
        """Substract the vector to another."""
        return Vec2(self.x - other.x, self.y - other.y)

    def __mul__(self, other):
        """Multiply the vector with another."""
        if isinstance(other, Vec2):
            #  Dot product
            return self.x * other.x + self.y * other.y
        elif isinstance(other, float) or isinstance(other, int):
            #  Scalar product
            return Vec2(self.x * other, self.y * other, self.z * other)
        else:
            raise TypeError()

    def __truediv__(self, other):
        """Divide the vector to another."""
        if isinstance(other, Vec2):
            #  Dot product
            return self.x / other.x + self.y / other.y
        elif isinstance(other, float) or isinstance(other, int):
            #  Scalar product
            return Vec2(self.x / other, self.y / other)
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
        return math.sqrt(self.x * self.x + self.y * self.y)

    def normalize(self):
        """Normalize the vector."""
        return self / self.norm()
