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

"""Utility functions for the shapely module."""

from shapely.geometry import LineString
from shapely.geometry import MultiPolygon
from shapely.geometry import Point
from shapely.geometry import Polygon

from utils.vector import Vector2D


def cut_line_string(line, distance):
    """Cut a LineString at some distance."""
    # http://stackoverflow.com/questions/31072945/shapely-cut-a-piece-from-a-linestring-at-two-cutting-points
    assert isinstance(line, LineString)
    if distance <= 0.0 or distance >= line.length:
        return [LineString(line)]
    coords = list(line.coords)
    for i, p in enumerate(coords):
        pd = line.project(Point(p))
        if pd == distance:
            return [
                LineString(coords[:i + 1]),
                LineString(coords[i:])]
        if pd > distance:
            cp = line.interpolate(distance)
            return [
                LineString(coords[:i] + [(cp.x, cp.y)]),
                LineString([(cp.x, cp.y)] + coords[i:])
            ]


def cut_piece_line_string(line, distance, length):
    """Cut a LineString from some distance along some length."""
    # http://stackoverflow.com/questions/31072945/shapely-cut-a-piece-from-a-linestring-at-two-cutting-points
    assert isinstance(line, LineString)
    try:
        precut = cut_line_string(line, distance)[1]
        result = cut_line_string(precut, length)[0]
    except IndexError:
        return line
    return result


def invert_line_string(line):
    """Revert the coords order from a LineString."""
    assert isinstance(line, LineString)
    # http://stackoverflow.com/questions/33175022/order-of-linestrings-in-multilinestring-object-in-shapely
    return LineString(line.coords[::-1])


def simplify_polygon(polygon, tolerance=0.01):
    """Remove doubles coords from a polygon."""
    assert isinstance(polygon, Polygon) or isinstance(polygon, MultiPolygon)
    # Get the coordinates
    coords = []
    if isinstance(polygon, Polygon):
        coords = polygon.exterior.coords
    elif isinstance(polygon, MultiPolygon):
        for geom in polygon.geoms:
            coords += geom.exterior.coords
    else:
        return None
    # remove the doubled coordinates
    newCoords = []
    v0 = Vector2D(float('inf'), float('inf'))
    for coord in coords:
        v = Vector2D(coord[0], coord[1])
        if (v0 - v).norm() > tolerance:
            v0 = v
            newCoords += [[coord[0], coord[1]]]
    return Polygon(newCoords)


def convert_polygon_to_vector2d_list(polygon):
    """Convert a shapely polygon to a list of Vector2D."""
    assert isinstance(polygon, Polygon) or isinstance(polygon, MultiPolygon)
    coords = []
    if isinstance(polygon, Polygon):
        coords = polygon.exterior.coords
    elif isinstance(polygon, MultiPolygon):
        for geom in polygon.geoms:
            coords += geom.exterior.coords
    else:
        return None
    return [Vector2D(x, y) for (x, y) in coords]


def intersects(a1, b1, a2, b2, tolerance=0.05):
    """Compute the intersection of a segment a1, b1 with a segment a2, b2. a1, a2, b1 and b2 are vectors. Returns a Vector2D."""
    # src: http://stackoverflow.com/questions/563198/how-do-you-detect-where-two-line-segments-intersect
    # Note: Almost the same can be achieved using LineString.intersection(), but the coplanar bounds are
    #       rejected. For this reason, a custom algorithm is preferred there.

    assert isinstance(a1, Vector2D)
    assert isinstance(b1, Vector2D)
    assert isinstance(a2, Vector2D)
    assert isinstance(b2, Vector2D)

    ls1 = LineString([[a1.x, a1.y], [b1.x, b1.y]]).buffer(tolerance)
    ls2 = LineString([[a2.x, a2.y], [b2.x, b2.y]]).buffer(tolerance)

    intersection = ls1.intersection(ls2)
    if intersection is not None and len(intersection.centroid.coords) == 1:
        return Vector2D(intersection.centroid.coords[0][0], intersection.centroid.coords[0][1])

    return None
