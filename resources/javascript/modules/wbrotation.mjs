/*
 * assumptions: a rotation is a table with x, y, z and a ids
 *              and angle is in radians.
 *              e.g. r = { x = 0; y = 1; z = 0; a = 1.5708 }
 *              a quaternion is a table with w, x, y, and z ids
 *              a vector is a table with x, y and z ids
 *              a matrix 3x3 is a table with 9 elements
 */

export function testFunction() {
  return 'WORKS';
};

export function equal(rA, rB) {
  return rA.x === rB.x && rA.y === rB.y && rA.z === rB.z && rA.a === rB.a;
};

export function fromquaternion(q) {
  var r = {};

  r['a'] = 2 * Math.acos(q.w);
  if (r.a < 0.0001) {
    r['x'] = 0;
    r['y'] = 1;
    r['z'] = 0;
    r['a'] = 0;
  } else {
    // normalize axes
    var n = Math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z);
    r['x'] = q.x / n;
    r['y'] = q.y / n;
    r['z'] = q.z / n;
  }

  return r;
};

export function frommatrix3 (m) {
  var r = {};
  var cosAngle = 0.5 * (m[1] + m[5] + m[9] - 1);
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

export function toquaternion (r) {
  normalize(r);
  var halfAngle = rot.a * 0.5
  var sinHalfAngle = Math.sin(halfAngle);
  var cosHalfAngle = Math.cos(halfAngle);

  return {w: cosHalfAngle, x: rot.x * sinHalfAngle, y: rot.y * sinHalfAngle, z: rot.z * sinHalfAngle};
};

export function tomatrix3(r) {
  var c = Math.cos(r.a)
  var s = Math.sin(r.a)
  var t1 = 1 - c;
  var t2 = r.x * r.z * t1;
  var t3 = r.x * r.y * t1;
  var t4 = r.y * r.z * t1;

  var m = {};
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

export function isvalid(r) {
  return r.x !== 0 || r.y !== 0 || r.z !== 0;
}

export function isidentity(r) {
  return r.a === 0.0;
}

export function normalizeangle (r) {
  while (r.a < -Math.PI)
    r.a = r.a + 2 * Math.PI;
  while (r.a > Math.PI)
    r.a = r.a - 2 * Math.PI;
};

export function normalizeaxis (r) {
  if (!isvalid(r)) { // TODO: should give error instead of overwriting?
    r.x = 0;
    r.y = 1;
    r.z = 0;
  }

  var invl = 1 / Math.sqrt(r.x * r.x + r.y * r.y + r.z * r.z);
  r.x = r.x * invl;
  r.y = r.y * invl;
  r.z = r.z * invl;
};

export function normalize(r) {
  normalizeaxis(r);
  normalizeangle(r);
  return r;
};


export function combine(rA, rB) {
  var qA = toquaternion(rA);
  var qB = toquaternion(rB);

  var q = {
    w: qA.w * qB.w - qA.x * qB.x - qA.y * qB.y - qA.z * qB.z,
    x: qA.w * qB.x + qA.x * qB.w + qA.y * qB.z - qA.z * qB.y,
    y: qA.w * qB.y - qA.x * qB.z + qA.y * qB.w + qA.z * qB.x,
    z: qA.w * qB.z + qA.x * qB.y - qA.y * qB.x + qA.z * qB.w
  }
  return fromquaternion(q);
};

export function rotatevector3bymatrix3(m, v) {
  var vector3 = {
    x: m[1] * v.x + m[2] * v.y + m[3] * v.z,
    y: m[4] * v.x + m[5] * v.y + m[6] * v.z,
    z: m[7] * v.x + m[8] * v.y + m[9] * v.z
  }

  return vector3;
};

export function rotatevector3byrotation(r, v) {
  var matrix3 = tomatrix3(r);
  var vector3 = rotatevector3bymatrix3(matrix3, v);

  return vector3;
};

export function rotatevector3byquaternion(q, v) {
  var rotation = fromquaternion(q);
  var vector3 = rotatevector3byrotation(rotation, v);
  return vector3;
};
