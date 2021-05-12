/*
 * content: utility functions for rotations
 * assumptions: a rotation is an object with x, y, z and a keys
 *              and angle is in radians.
 *              e.g. r = {x: 0, y: 1, z: 0, a: 1.5708}
 *              a quaternion is an object with w, x, y, and z keys
 *              a vector is an object with x, y and z keys
 *              a matrix 3x3 is an object with 9 elements
 */

export function testFunction() { // TODO: to remove
  return 'WBROTATION WORKS';
};

export function equal(rA, rB) {
  return rA.x === rB.x && rA.y === rB.y && rA.z === rB.z && rA.a === rB.a;
};

export function fromQuaternion(q) {
  let r = {};

  r['a'] = 2 * Math.acos(q.w);
  if (r.a < 0.0001) {
    r['x'] = 0;
    r['y'] = 1;
    r['z'] = 0;
    r['a'] = 0;
  } else {
    // normalize axes
    let n = Math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z);
    r['x'] = q.x / n;
    r['y'] = q.y / n;
    r['z'] = q.z / n;
  }

  return r;
};

export function fromMatrix3(m) {
  let r = {};
  const cosAngle = 0.5 * (m[1] + m[5] + m[9] - 1);
  if (Math.abs(cosAngle) > 1) {
    r['x'] = 1;
    r['y'] = 0;
    r['z'] = 0;
    r['a'] = 0;
  } else {
    r['x'] = m[8] - m[6];
    r['y'] = m[3] - m[7];
    r['z'] = m[4] - m[2];
    r['a'] = Math.acos(cosAngle);
  }

  return r;
};

export function toQuaternion(r) {
  normalize(r);
  let halfAngle = r.a * 0.5;
  let sinHalfAngle = Math.sin(halfAngle);
  let cosHalfAngle = Math.cos(halfAngle);

  return {w: cosHalfAngle, x: r.x * sinHalfAngle, y: r.y * sinHalfAngle, z: r.z * sinHalfAngle};
};

export function toMatrix3(r) {
  const c = Math.cos(r.a);
  const s = Math.sin(r.a);
  const t1 = 1 - c;
  const t2 = r.x * r.z * t1;
  const t3 = r.x * r.y * t1;
  const t4 = r.y * r.z * t1;

  let m = {};
  m[0] = r.x * r.x * t1 + c;
  m[1] = t3 - r.z * s;
  m[2] = t2 + r.y * s;
  m[3] = t3 + r.z * s;
  m[4] = r.y * r.y * t1 + c;
  m[5] = t4 - r.x * s;
  m[6] = t2 - r.y * s;
  m[7] = t4 + r.x * s;
  m[8] = r.z * r.z * t1 + c;

  return m;
};

export function isValid(r) {
  return r.x !== 0 || r.y !== 0 || r.z !== 0;
}

export function isIdentity(r) {
  return r.a === 0.0;
}

export function normalizeAngle(r) {
  while (r.a < -Math.PI)
    r.a = r.a + 2 * Math.PI;
  while (r.a > Math.PI)
    r.a = r.a - 2 * Math.PI;
};

export function normalizeAxis(r) {
  if (!isValid(r)) { // TODO: should give error instead of overwriting?
    r.x = 0;
    r.y = 1;
    r.z = 0;
  }

  let invl = 1 / Math.sqrt(r.x * r.x + r.y * r.y + r.z * r.z);
  r.x = r.x * invl;
  r.y = r.y * invl;
  r.z = r.z * invl;
};

export function normalize(r) {
  normalizeAxis(r);
  normalizeAngle(r);
  return r;
};

export function combine(rA, rB) {
  const qA = toQuaternion(rA);
  const qB = toQuaternion(rB);

  let q = {
    w: qA.w * qB.w - qA.x * qB.x - qA.y * qB.y - qA.z * qB.z,
    x: qA.w * qB.x + qA.x * qB.w + qA.y * qB.z - qA.z * qB.y,
    y: qA.w * qB.y - qA.x * qB.z + qA.y * qB.w + qA.z * qB.x,
    z: qA.w * qB.z + qA.x * qB.y - qA.y * qB.x + qA.z * qB.w
  };
  return fromQuaternion(q);
};

export function rotateVector3ByMatrix3(m, v) {
  const vector3 = {
    x: m[1] * v.x + m[2] * v.y + m[3] * v.z,
    y: m[4] * v.x + m[5] * v.y + m[6] * v.z,
    z: m[7] * v.x + m[8] * v.y + m[9] * v.z
  };

  return vector3;
};

export function rotateVector3ByRotation(r, v) {
  const matrix3 = toMatrix3(r);
  const vector3 = rotateVector3ByMatrix3(matrix3, v);

  return vector3;
};

export function rotateVector3ByQuaternion(q, v) {
  const rotation = fromQuaternion(q);
  const vector3 = rotateVector3ByRotation(rotation, v);

  return vector3;
};
