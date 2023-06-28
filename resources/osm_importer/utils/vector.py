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

"""Vector2D class container."""

import math


class Vector2D(object):
    """Abstraction of mathematical 2D vector."""

    __slots__ = ['x', 'y']

    def __init__(self, x, y):
        """Constructor: set the vector coordinate."""
        self.x = x
        self.y = y

    def __repr__(self):
        """To String conversion."""
        return 'Vector2D({}, {})'.format(self.x, self.y)

    def __eq__(self, other):
        """Equality operator."""
        if isinstance(other, Vector2D):
            return self.x == other.x and self.y == other.y
        return False

    def __ne__(self, other):
        """Equality operator."""
        return not self == other

    def __add__(self, other):
        """Add the vector with another."""
        return Vector2D(self.x + other.x, self.y + other.y)

    def __sub__(self, other):
        """Substract the vector to another."""
        return Vector2D(self.x - other.x, self.y - other.y)

    def __mul__(self, other):
        """Multiply the vector with another."""
        if isinstance(other, Vector2D):
            #  Dot product
            return self.x * other.x + self.y * other.y
        elif isinstance(other, float):
            #  Scalar product
            return Vector2D(self.x * other, self.y * other)
        else:
            raise TypeError()

    def __truediv__(self, other):
        """Divide the vector to another."""
        if isinstance(other, Vector2D):
            #  Dot product
            return self.x / other.x + self.y / other.y
        elif isinstance(other, float):
            #  Scalar product
            return Vector2D(self.x / other, self.y / other)
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

    def angle(self, other=None):
        """Compute the vector angle."""
        if other is not None:
            a = math.atan2(other.y, other.x) - math.atan2(self.y, self.x)
            a = ((a + math.pi) % (2.0 * math.pi)) - math.pi  # keep the [-pi, pi] range
            return a
        else:
            return math.atan2(self.y, self.x)
