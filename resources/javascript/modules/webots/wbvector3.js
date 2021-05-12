/*
 * content: utility functions for vector3 variables
 * assumptions: a vector 3 is an object with x, y and z keys
 *              e.g. v = {x: 1, y: 2, z: 3}
 */

export function testFunction() { // TODO: to remove
  return 'WBVECTOR3 WORKS';
};

export function equal(vA, vB) {
  return vA.x === vB.x && vA.y === vB.y && vA.z === vB.z;
};

export function add(vA, vB) {
  return {x: vA.x + vB.x, y: vA.y + vB.y, z: vA.z + vB.z};
};

export function minus(vA, vB) {
  return {x: vA.x - vB.x, y: vA.y - vB.y, z: vA.z - vB.z};
};

export function multiply(v, s) {
  return {x: s * v.x, y: s * v.y, z: s * v.z};
};

export function norm(v) {
  return Math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
};

export function distance(vA, vB) {
  return norm(minus(vA, vB));
};

export function cross(vA, vB) {
  return {x: vA.y * vB.z - vA.z * vB.y, y: vA.z * vB.x - vA.x * vB.z, z: vA.x * vB.y - vA.y * vB.x};
};

export function dot(vA, vB) {
  return vA.x * vB.x + vA.y * vB.y + vA.z * vB.z;
};

export function normalize(v) {
  const n = norm(v);
  if (n === 0)
    error();
  else
    return {x: v.x / n, y: v.y / n, z: v.z / n};
}
