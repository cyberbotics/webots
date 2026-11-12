# Copyright 1996-2024 Cyberbotics Ltd.
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

"""Offline regression tests for OSM elevation coordinates."""

import math
import re
import sys
import subprocess
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from elevation import Elevation  # noqa: E402
from projection import Projection  # noqa: E402


class TestElevation(unittest.TestCase):
    """Check exported terrain and interpolation in the same projected XY frame."""

    def setUp(self):
        """Use the real projection and replace only the remote elevation service."""
        Projection.initProjection(6.55, 46.51, '')
        self.projection = Projection.getProjection()
        self.bounds = dict(minlat=46.51, minlon=6.55, maxlat=46.512, maxlon=6.551)
        self.xmin, self.ymin = self.projection(self.bounds['minlon'], self.bounds['minlat'])
        self.xmax, self.ymax = self.projection(self.bounds['maxlon'], self.bounds['maxlat'])
        self.offset = None

        def elevations(locations, key):
            result = [0.1 * x + 0.2 * y for x, y in
                      (self.projection(lon, lat) for lat, lon in locations)]
            self.offset = min(result)
            return result

        with patch.object(Elevation, 'get_elevation_from_geonames', side_effect=elevations):
            self.elevation = Elevation(self.projection, **self.bounds)

    def expected_height(self, x, y):
        """Height of the synthetic inclined plane after minimum-height offset."""
        return 0.1 * x + 0.2 * y - self.offset

    def test_exported_grid_coordinates(self):
        """Every exported grid vertex must coincide with its sampled XY location."""
        text = self.elevation.floorString
        translation = [float(v) for v in re.search(r'translation ([^\n]+)', text)[1].split()]
        nx = int(re.search(r'xDimension (\d+)', text)[1])
        ny = int(re.search(r'yDimension (\d+)', text)[1])
        dx = float(re.search(r'xSpacing ([^\n]+)', text)[1])
        dy = float(re.search(r'ySpacing ([^\n]+)', text)[1])
        heights = [float(v) for v in re.search(r'height \[([^]]+)', text)[1].split()]
        self.assertAlmostEqual((nx - 1) * dx, self.xmax - self.xmin)
        self.assertAlmostEqual((ny - 1) * dy, self.ymax - self.ymin)
        self.assertAlmostEqual(translation[0], -(self.xmax - self.xmin) / 2, delta=0.005)
        self.assertAlmostEqual(translation[1], -(self.ymax - self.ymin) / 2, delta=0.005)
        self.assertEqual(len(heights), nx * ny)
        for j in range(ny):
            for i in range(nx):
                self.assertAlmostEqual(heights[i + j * nx],
                                       self.expected_height(self.xmin + i * dx, self.ymin + j * dy), delta=0.003)

    def test_interpolated_heights(self):
        """Interior points, grid edges and corners retain the plane's height."""
        for fx, fy in ((0.25, 0.25), (0.75, 0.75), (0, 0), (0, 1), (1, 0), (1, 1), (0, 0.5), (0.5, 0)):
            x = self.xmin + fx * (self.xmax - self.xmin)
            y = self.ymin + fy * (self.ymax - self.ymin)
            with self.subTest(fx=fx, fy=fy):
                actual = self.elevation.interpolate_height(x, y)
                self.assertTrue(math.isfinite(actual))
                self.assertAlmostEqual(actual, self.expected_height(x, y), delta=0.003)

    def test_nonplanar_cell_triangles(self):
        """Interpolate the lower-left to upper-right diagonal used by the mesh."""
        elevation = Elevation.__new__(Elevation)
        elevation.elevationArray = [
            {'x': 0, 'y': 0, 'height': 0},
            {'x': 4, 'y': 0, 'height': 4},
            {'x': 0, 'y': 2, 'height': 8},
            {'x': 4, 'y': 2, 'height': 20},
        ]
        # Lower triangle: h = x + 8*y. Upper triangle: h = 3*x + 4*y.
        for x, y, expected in ((3, 0.5, 7), (1, 1.5, 9), (2, 1, 10), (0, 1, 4), (2, 0, 2)):
            with self.subTest(x=x, y=y):
                self.assertAlmostEqual(elevation.interpolate_height(x, y), expected)

    def test_imported_building_height(self):
        """Run the real XML importer and building exporter, without network access."""
        importer = Path(__file__).resolve().parents[1] / 'importer.py'
        nodes = ((6.5502, 46.5104), (6.5508, 46.5104), (6.5508, 46.5116), (6.5502, 46.5116))
        xml = '<osm><bounds minlat="46.51" minlon="6.55" maxlat="46.512" maxlon="6.551"/>'
        for i, (lon, lat) in enumerate(nodes):
            xml += f'<node id="{i}" lon="{lon}" lat="{lat}"/>'
        xml += '<way id="10">' + ''.join(f'<nd ref="{i}"/>' for i in (0, 1, 2, 3, 0))
        xml += '<tag k="building" v="yes"/><tag k="addr:country" v="CH"/></way></osm>'
        script = """
import runpy
import sys
from unittest.mock import patch
sys.path.insert(0, sys.argv[1])
from elevation import Elevation
from projection import Projection

def elevations(locations, key):
    projection = Projection.getProjection()
    return [0.1 * x + 0.2 * y for x, y in (projection(lon, lat) for lat, lon in locations)]

path = sys.argv[1] + '/importer.py'
sys.argv = [path] + sys.argv[2:]
with patch.object(Elevation, 'get_elevation_from_geonames', side_effect=elevations):
    with patch('urllib.request.urlopen', side_effect=AssertionError('Unexpected network request')):
        runpy.run_path(path, run_name='__main__')
"""
        expected = min(self.expected_height(*self.projection(lon, lat)) for lon, lat in nodes)
        with tempfile.TemporaryDirectory() as directory:
            input_path = Path(directory) / 'map.osm'
            input_path.write_text(xml)
            for enabled in (False, True):
                output_path = Path(directory) / f'map-{enabled}.wbt'
                arguments = [sys.executable, '-c', script, str(importer.parent),
                             '--input=' + str(input_path), '--output=' + str(output_path)]
                if enabled:
                    arguments.append('--enable-3D')
                result = subprocess.run(arguments, capture_output=True, text=True, timeout=30)
                self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
                world = output_path.read_text()
                translation = re.search(r'SimpleBuilding \{\s+translation ([^\n]+)', world)
                self.assertIsNotNone(translation, world)
                height = float(translation[1].split()[2])
                self.assertAlmostEqual(height, expected if enabled else 0, delta=0.01)
                self.assertEqual('geometry ElevationGrid' in world, enabled)

    def test_outside_grid(self):
        """Coordinates outside the acquired grid keep the existing zero fallback."""
        for x, y in ((self.xmin - 1, self.ymin), (self.xmax + 1, self.ymax),
                     (self.xmin, self.ymin - 1), (self.xmax, self.ymax + 1)):
            self.assertEqual(self.elevation.interpolate_height(x, y), 0)


if __name__ == '__main__':
    unittest.main()
