/*
 * content: utility functions for vector3 variables
 * assumptions: a vector3 is an object with x, y and z keys
 *              e.g. v = {x: 1, y: 2, z: 3}
 */

import * as wbutility from './wbutility.js';

export function equal(vA, vB) {
  wbutility.assert(wbutility.isVector3(vA), 'Expected an object with keys (x, y, z) as first parameter in wbvector3.equal.');
  wbutility.assert(wbutility.isVector3(vB), 'Expected an object with keys (x, y, z) as second parameter in wbvector3.equal.');

  return vA.x === vB.x && vA.y === vB.y && vA.z === vB.z;
};

export function add(vA, vB) {
  wbutility.assert(wbutility.isVector3(vA), 'Expected an object with keys (x, y, z) as first parameter in wbvector3.add.');
  wbutility.assert(wbutility.isVector3(vB), 'Expected an object with keys (x, y, z) as second parameter in wbvector3.add.');

  return {x: vA.x + vB.x, y: vA.y + vB.y, z: vA.z + vB.z};
};

export function minus(vA, vB) {
  wbutility.assert(wbutility.isVector3(vA), 'Expected an object with keys (x, y, z) as first parameter in wbvector3.minus.');
  wbutility.assert(wbutility.isVector3(vB), 'Expected an object with keys (x, y, z) as second parameter in wbvector3.minus.');

  return {x: vA.x - vB.x, y: vA.y - vB.y, z: vA.z - vB.z};
};

export function multiply(v, s) {
  wbutility.assert(wbutility.isVector3(v), 'Expected an object with keys (x, y, z) as first parameter in wbvector3.multiply.');
  wbutility.assert(wbutility.isScalar(s), 'Expected a number as second parameter in wbvector3.multiply.');

  return {x: s * v.x, y: s * v.y, z: s * v.z};
};

export function norm(v) {
  wbutility.assert(wbutility.isVector3(v), 'Expected an object with keys (x, y, z) as parameter in wbvector3.norm.');

  return Math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
};

export function distance(vA, vB) {
  wbutility.assert(wbutility.isVector3(vA), 'Expected an object with keys (x, y, z) as first parameter in wbvector3.distance.');
  wbutility.assert(wbutility.isVector3(vB), 'Expected an object with keys (x, y, z) as second parameter in wbvector3.distance.');

  return norm(minus(vA, vB));
};

export function cross(vA, vB) {
  wbutility.assert(wbutility.isVector3(vA), 'Expected an object with keys (x, y, z) as first parameter in wbvector3.cross.');
  wbutility.assert(wbutility.isVector3(vB), 'Expected an object with keys (x, y, z) as second parameter in wbvector3.cross.');

  return {x: vA.y * vB.z - vA.z * vB.y, y: vA.z * vB.x - vA.x * vB.z, z: vA.x * vB.y - vA.y * vB.x};
};

export function dot(vA, vB) {
  wbutility.assert(wbutility.isVector3(vA), 'Expected an object with keys (x, y, z) as first parameter in wbvector3.dot.');
  wbutility.assert(wbutility.isVector3(vB), 'Expected an object with keys (x, y, z) as second parameter in wbvector3.dot.');

  return vA.x * vB.x + vA.y * vB.y + vA.z * vB.z;
};

export function normalize(v) {
  wbutility.assert(wbutility.isVector3(v), 'Expected an object with keys (x, y, z) as parameter in wbvector3.normalize.');

  const n = norm(v);
  if (n === 0) {
    wbutility.error('The norm cannot be zero when normalizing.');
    return null;
  } else
    return {x: v.x / n, y: v.y / n, z: v.z / n};
}
