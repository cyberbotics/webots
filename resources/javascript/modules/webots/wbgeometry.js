/*
 * content:
 */
import * as wbvector2 from 'wbvector2.js';

export function testFunction() { // TODO: to remove
  return 'WBGEOMETRY WORKS';
};

// create a list of 'div' circle coordinates according
// to a circle centered at {'cx', 'cy'} and rotated by
// 'shift' radians
export function circle(radius, div, cx, cy, shift) {
  let circle = [];
  const quantum = 2 * Math.PI / div;

  for (let i = 0; i <= div; ++i)
    circle.push({x: radius * Math.cos(i * quantum + shift) + cx, y: radius * Math.sin(i * quantum + shift) + cy});

  return circle;
};

// determine if a point is inside a given polygon or not
// polygon is a list of (x,y) pairs.
export function isPointInPolygon(x, y, polygon) {
  // local wbcore = require('wbcore')
  // local n = wbcore.tablelength(polygon)
  const n = polygon.length;
  if (n < 3)
    return false;

  let inside = false;
  let p1x = polygon[0].x;
  let p1y = polygon[0].y;
  let p2x;
  let p2y;
  let xinters = 0;
  for (let i = 1; i <= n; ++i) {
    p2x = polygon[i % n + 1].x;
    p2y = polygon[i % n + 1].y;

    if (y > Math.min(p1y, p2y)) {
      if (y <= Math.max(p1y, p2y)) {
        if (x <= Math.max(p1x, p2x)) {
          if (p1y !== p2y)
            xinters = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x;
          if (p1x === p2x || x <= xinters)
            inside = !inside;
        }
      }
    }

    p1x = p2x;
    p1y = p2y;
  }

  return inside;
};

// return the closest points in the array
// referencepoint should be a 2d point (x, y)
// pointsarray should be a array of 2d points (x, y)
export function findClosest2DPointInArray(referencePoint, pointsArray) {
  const pointsNumber = pointsArray.length;
  let point = referencePoint;
  let dist = Infinity;
  let currentDistance;
  for (let i = 0; i < pointsNumber; ++i) {
    currentDistance = wbvector2.distance(referencePoint, pointsArray[i]);
    if (currentDistance < dist) {
      dist = currentDistance;
      point = pointsArray[i];
    }
  }

  return point;
};

// check if an array of points (x, y) is defined in a clockwise order
// based on the formula (x2-x1)(y2+y1) (http://stackoverflow.com/questions/1165647/how-to-determine-if-a-list-of-polygon-points-are-in-clockwise-order)
export function isListClockwise2D(points) {
  const pointsNumber = points.length;
  let total = 0;

  for (let i = 0; i < pointsNumber - 1; ++i)
    total = total + (points[i + 1].x - points[i].x) * (points[i + 1].y + points[i].y);
  total = total + (points[0].x - points[pointsNumber - 1].x) * (points[0].y + points[pointsNumber - 1].y);

  if (total >= 0)
    return false;
  else
    return true;
};

// create a B-Spline curve of third order using the array of points (x and y)
// return then a new array of point following this B-Spline subdividing
// each segment by 'subdivision'
export function bSpline2D(points, subdivision) {
  local spline = {}
  const pointsNumber = points.length;

  // extend the points array
  points.unshift(wbvector2.add(points[0], wbvector2.minus(points[0], points[1])));
  points.unshift(wbvector2.add(points[0], wbvector2.minus(points[0], points[1])));
  points.push(wbvector2.add(points[pointsNumber - 1], wbvector2.minus(points[pointsNumber - 1], points[pointsNumber - 2])));
  points.push(wbvector2.add(points[pointsNumber], wbvector2.minus(points[pointsNumber], points[pointsNumber - 1])));

  // Interpolation
  let index = 0;
  spline[index] = points[2]; // first point
  for i = 0, pointsNumber - 1 do
    let a = {}; // compute the third order coefficients for x
    let b = {}; // compute the third order coefficients for y
    a[1] = (-points[i-1].x + 3 * points[i].x - 3 * points[i+1].x + points[i+2].x) / 6.0;
    a[2] = (3 * points[i-1].x - 6 * points[i].x + 3 * points[i+1].x) / 6.0;
    a[3] = (-3 * points[i-1].x + 3 * points[i+1].x) / 6.0;
    a[4] = (points[i-1].x + 4 * points[i].x + points[i+1].x) / 6.0;
    b[1] = (-points[i-1].y + 3 * points[i].y - 3 * points[i+1].y + points[i+2].y) / 6.0;
    b[2] = (3 * points[i-1].y - 6 * points[i].y + 3 * points[i+1].y) / 6.0;
    b[3] = (-3 * points[i-1].y + 3 * points[i+1].y) / 6.0;
    b[4] = (points[i-1].y + 4 * points[i].y + points[i+1].y) / 6.0;
    for j = 1, subdivision do
      index = index + 1
      spline[index] = {}
      local t = j / subdivision
      spline[index].x = (a[3] + t * (a[2] + t * a[1])) * t + a[4]
      spline[index].y = (b[3] + t * (b[2] + t * b[1])) * t + b[4]
    end
  end
  return spline
}

/*
-- create a B-Spline curve of third order using the array of points (x, y and z)
-- return then a new array of point following this B-Spline subdividing
-- each segment by 'subdivision'
function M.bspline3D(points, subdivision)
  local spline = {}
  local wbcore = require('wbcore')
  local wbvector3 = require('wbvector3')
  local pointsNumber = wbcore.tablelength(points)

  -- extend the points array
  points[0]                = wbvector3.add(points[1], wbvector3.minus(points[1], points[2]))
  points[-1]               = wbvector3.add(points[0], wbvector3.minus(points[0], points[1]))
  points[pointsNumber + 1] = wbvector3.add(points[pointsNumber], wbvector3.minus(points[pointsNumber], points[pointsNumber - 1]))
  points[pointsNumber + 2] = wbvector3.add(points[pointsNumber + 1], wbvector3.minus(points[pointsNumber + 1], points[pointsNumber]))

  -- Interpolation
  local index = 1
  spline[index] = points[1] -- first point
  for i = 1, pointsNumber - 1 do
    local a = {} -- compute the third order coefficients for x
    local b = {} -- compute the third order coefficients for y
    local c = {} -- compute the third order coefficients for z
    a[1] = (-points[i-1].x + 3 * points[i].x - 3 * points[i+1].x + points[i+2].x) / 6.0;
    a[2] = (3 * points[i-1].x - 6 * points[i].x + 3 * points[i+1].x) / 6.0;
    a[3] = (-3 * points[i-1].x + 3 * points[i+1].x) / 6.0;
    a[4] = (points[i-1].x + 4 * points[i].x + points[i+1].x) / 6.0;
    b[1] = (-points[i-1].y + 3 * points[i].y - 3 * points[i+1].y + points[i+2].y) / 6.0;
    b[2] = (3 * points[i-1].y - 6 * points[i].y + 3 * points[i+1].y) / 6.0;
    b[3] = (-3 * points[i-1].y + 3 * points[i+1].y) / 6.0;
    b[4] = (points[i-1].y + 4 * points[i].y + points[i+1].y) / 6.0;
    c[1] = (-points[i-1].z + 3 * points[i].z - 3 * points[i+1].z + points[i+2].z) / 6.0;
    c[2] = (3 * points[i-1].z - 6 * points[i].z + 3 * points[i+1].z) / 6.0;
    c[3] = (-3 * points[i-1].z + 3 * points[i+1].z) / 6.0;
    c[4] = (points[i-1].z + 4 * points[i].z + points[i+1].z) / 6.0;
    for j = 1, subdivision do
      index = index + 1
      spline[index] = {}
      local t = j / subdivision
      spline[index].x = (a[3] + t * (a[2] + t * a[1])) * t + a[4]
      spline[index].y = (b[3] + t * (b[2] + t * b[1])) * t + b[4]
      spline[index].z = (c[3] + t * (c[2] + t * c[1])) * t + c[4]
    end
  end
  return spline
end

return M
*/
