/*
 * content: utility functions for geometry based operations
 */

import * as wbvector2 from './wbvector2.js';
import * as wbvector3 from './wbvector3.js';
import * as wbutility from './wbutility.js';

// create an array of 'div' circle coordinates according to a circle centered at 'center' and rotated by 'shift' radians
export function circle(radius, div, center, shift) {
  wbutility.assert(wbutility.isScalar(radius) && radius > 0, 'Expected radius to be a positive number in wbgeometry.circle.');
  wbutility.assert(wbutility.isScalar(div) && div > 0, 'Expected div to be a positive number in wbgeometry.circle.');
  wbutility.assert(wbutility.isVector2(center), 'Expected c to be an object with keys (x, y) in wbgeometry.circle.');
  wbutility.assert(wbutility.isScalar(shift), 'Expected shift to be a number in wbgeometry.circle.');

  let circle = [];
  const quantum = 2 * Math.PI / div;

  for (let i = 0; i <= div; ++i)
    circle.push({x: radius * Math.cos(i * quantum + shift) + center.x, y: radius * Math.sin(i * quantum + shift) + center.y});

  return circle;
};

// determine if a point is inside a given polygon or not. The polygon is an array of objects with keys (x, y)
export function isPoint2InPolygon(p, polygon) {
  wbutility.assert(wbutility.isVector2(p), 'Expected an object with keys (x, y) as first parameter in wbgeometry.isPointInPolygon.');
  wbutility.assert(wbutility.isArrayOfPoints(polygon, 2), 'Expected an array of objects with keys (x, y) as second parameter in wbgeometry.isPointInPolygon.');

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
    p2x = polygon[i % n].x;
    p2y = polygon[i % n].y;

    if (p.y > Math.min(p1y, p2y)) {
      if (p.y <= Math.max(p1y, p2y)) {
        if (p.x <= Math.max(p1x, p2x)) {
          if (p1y !== p2y)
            xinters = (p.y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x;
          if (p1x === p2x || p.x <= xinters)
            inside = !inside;
        }
      }
    }

    p1x = p2x;
    p1y = p2y;
  }

  return inside;
};

// return the closest point in the array to the reference point. The array is comprised of objects with keys (x, y)
export function findClosestPoint2InArray(reference, points) {
  wbutility.assert(wbutility.isVector2(reference), 'Expected an object with keys (x, y) as first parameter in wbgeometry.findClosestPoint2InArray.');
  wbutility.assert(wbutility.isArrayOfPoints(points, 2), 'Expected an array of objects with keys (x, y) as second parameter in wbgeometry.findClosestPoint2InArray.');

  let point = reference;
  let dist = Infinity;
  let currentDistance;
  for (let i = 0; i < points.length; ++i) {
    currentDistance = wbvector2.distance(reference, points[i]);
    if (currentDistance < dist) {
      dist = currentDistance;
      point = points[i];
    }
  }

  return point;
};

// check if an array of points (x, y) is defined in a clockwise order
// based on the formula (x2-x1)(y2+y1) (http://stackoverflow.com/questions/1165647/how-to-determine-if-a-list-of-polygon-points-are-in-clockwise-order)
export function isPoint2ArrayClockwise(points) {
  wbutility.assert(wbutility.isArrayOfPoints(points, 2), 'Expected an array of objects with keys (x, y) as second parameter in wbgeometry.findClosestPoint2InArray.');

  const n = points.length;
  let total = 0;

  for (let i = 0; i < n - 1; ++i)
    total = total + (points[i + 1].x - points[i].x) * (points[i + 1].y + points[i].y);
  total = total + (points[0].x - points[n - 1].x) * (points[0].y + points[n - 1].y);

  if (total >= 0)
    return false;
  else
    return true;
};

// create a B-Spline curve of third order using the array of points (x and y)
// return then a new array of point following this B-Spline subdividing each segment by 'subdivision'
export function bSpline2(points, subdivision) {
  wbutility.assert(wbutility.isArrayOfPoints(points, 2), 'Expected an array of objects with keys (x and y) as first parameter in wbgeometry.bSpline2.');
  wbutility.assert(wbutility.isScalar(subdivision), 'Expected subdivision to be a number in wbgeometry.bSpline2.');

  let spline = [];
  const pointsNumber = points.length;
  // extend the points array
  points.push(wbvector2.add(points[pointsNumber - 1], wbvector2.minus(points[pointsNumber - 1], points[pointsNumber - 2])));
  points.push(wbvector2.add(points[pointsNumber], wbvector2.minus(points[pointsNumber], points[pointsNumber - 1])));
  points.unshift(wbvector2.add(points[0], wbvector2.minus(points[0], points[1])));
  points.unshift(wbvector2.add(points[0], wbvector2.minus(points[0], points[1])));
  // Interpolation
  spline[0] = points[2]; // first point
  for (let i = 1; i < pointsNumber + 1; ++i) {
    let a = []; // compute the third order coefficients for x
    let b = []; // compute the third order coefficients for y
    a[0] = (-points[i - 1].x + 3 * points[i].x - 3 * points[i + 1].x + points[i + 2].x) / 6.0;
    a[1] = (3 * points[i - 1].x - 6 * points[i].x + 3 * points[i + 1].x) / 6.0;
    a[2] = (-3 * points[i - 1].x + 3 * points[i + 1].x) / 6.0;
    a[3] = (points[i - 1].x + 4 * points[i].x + points[i + 1].x) / 6.0;
    b[0] = (-points[i - 1].y + 3 * points[i].y - 3 * points[i + 1].y + points[i + 2].y) / 6.0;
    b[1] = (3 * points[i - 1].y - 6 * points[i].y + 3 * points[i + 1].y) / 6.0;
    b[2] = (-3 * points[i - 1].y + 3 * points[i + 1].y) / 6.0;
    b[3] = (points[i - 1].y + 4 * points[i].y + points[i + 1].y) / 6.0;

    for (let j = 1; j <= subdivision; j++) {
      let newPoint = {};
      let t = j / subdivision;
      newPoint.x = (a[2] + t * (a[1] + t * a[0])) * t + a[3];
      newPoint.y = (b[2] + t * (b[1] + t * b[0])) * t + b[3];
      spline.push(newPoint);
    }
  }

  return spline;
};

// create a B-Spline curve of third order using the array of points (x, y and z)
// return then a new array of point following this B-Spline subdividing each segment by 'subdivision'
export function bSpline3(points, subdivision) {
  wbutility.assert(wbutility.isArrayOfPoints(points, 3), 'Expected an array of objects with keys (x, y and z) as first parameter in wbgeometry.bSpline3.');
  wbutility.assert(wbutility.isScalar(subdivision), 'Expected subdivision to be a number in wbgeometry.bSpline3.');

  let spline = [];
  const pointsNumber = points.length;
  // extend the points array
  points.push(wbvector3.add(points[pointsNumber - 1], wbvector3.minus(points[pointsNumber - 1], points[pointsNumber - 2])));
  points.push(wbvector3.add(points[pointsNumber], wbvector3.minus(points[pointsNumber], points[pointsNumber - 1])));
  points.unshift(wbvector3.add(points[0], wbvector3.minus(points[0], points[1])));
  points.unshift(wbvector3.add(points[0], wbvector3.minus(points[0], points[1])));
  // interpolation
  spline[0] = points[2]; // first point
  for (let i = 2; i < pointsNumber + 1; ++i) {
    let a = []; // compute the third order coefficients for x
    let b = []; // compute the third order coefficients for y
    let c = []; // compute the third order coefficients for z
    a[0] = (-points[i - 1].x + 3 * points[i].x - 3 * points[i + 1].x + points[i + 2].x) / 6.0;
    a[1] = (3 * points[i - 1].x - 6 * points[i].x + 3 * points[i + 1].x) / 6.0;
    a[2] = (-3 * points[i - 1].x + 3 * points[i + 1].x) / 6.0;
    a[3] = (points[i - 1].x + 4 * points[i].x + points[i + 1].x) / 6.0;
    b[0] = (-points[i - 1].y + 3 * points[i].y - 3 * points[i + 1].y + points[i + 2].y) / 6.0;
    b[1] = (3 * points[i - 1].y - 6 * points[i].y + 3 * points[i + 1].y) / 6.0;
    b[2] = (-3 * points[i - 1].y + 3 * points[i + 1].y) / 6.0;
    b[3] = (points[i - 1].y + 4 * points[i].y + points[i + 1].y) / 6.0;
    c[0] = (-points[i - 1].z + 3 * points[i].z - 3 * points[i + 1].z + points[i + 2].z) / 6.0;
    c[1] = (3 * points[i - 1].z - 6 * points[i].z + 3 * points[i + 1].z) / 6.0;
    c[2] = (-3 * points[i - 1].z + 3 * points[i + 1].z) / 6.0;
    c[3] = (points[i - 1].z + 4 * points[i].z + points[i + 1].z) / 6.0;

    for (let j = 1; j <= subdivision; ++j) {
      let newPoint = {};
      let t = j / subdivision;
      newPoint.x = (a[2] + t * (a[1] + t * a[0])) * t + a[3];
      newPoint.y = (b[2] + t * (b[1] + t * b[0])) * t + b[3];
      newPoint.z = (c[2] + t * (c[1] + t * c[0])) * t + c[3];
      spline.push(newPoint);
    }
  }

  return spline;
};
