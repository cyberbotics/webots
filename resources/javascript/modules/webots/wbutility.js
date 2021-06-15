/*
 * content: utility functions
 */

export function assert(statement, message) {
  if (!statement)
    error(message);
};

export function error(message) {
  stderr.push(message);
};

export function info(message) {
  stdout.push(message);
};

export function deepCopy(original) {
  return JSON.parse(JSON.stringify(original));
};

export function degreesToRadians(angle) {
  return angle * Math.PI / 180.0;
};

export function radiansToDegrees(angle) {
  return angle * 180.0 / Math.PI;
};

export function isScalar(s) {
  if (typeof s !== 'number')
    return false;

  return true;
};

export function isVector2(v) {
  if (typeof v !== 'object' && Object.keys(v).length !== 2)
    return false;

  if (!v.hasOwnProperty('x') || !v.hasOwnProperty('y'))
    return false;

  return true;
};

export function isVector3(v) {
  if (typeof v !== 'object' && Object.keys(v).length !== 3)
    return false;

  if (!v.hasOwnProperty('x') || !v.hasOwnProperty('y') || !v.hasOwnProperty('z'))
    return false;

  return true;
};

export function isAxisAngle(r) {
  if (typeof r !== 'object' && Object.keys(r).length !== 4)
    return false;

  if (!r.hasOwnProperty('x') || !r.hasOwnProperty('y') || !r.hasOwnProperty('z') || !r.hasOwnProperty('a'))
    return false;

  return true;
};

export function isQuaternion(q) {
  if (typeof q !== 'object' && Object.keys(q).length !== 4)
    return false;

  if (!q.hasOwnProperty('w') || !q.hasOwnProperty('x') || !q.hasOwnProperty('y') || !q.hasOwnProperty('z'))
    return false;

  return true;
};

export function isMatrix3(m) {
  if (typeof m !== 'object' && Object.keys(m).length !== 9)
    return false;

  for (let i = 0; i < 9; ++i) {
    if (!m.hasOwnProperty(i))
      return false;
  }

  return true;
};

export function isArrayOfPoints(array, dim) {
  if (!Array.isArray(array))
    return false;

  if (!(dim === 2 || dim === 3)) {
    error('The second paramter of wbutility.isArrayOfPoints can only be 2 or 3.');
    return false;
  }

  for (let i = 0; i < array.length; ++i) {
    if (dim === 2 && !isVector2(array[i]))
      return false;
    if (dim === 3 && !isVector3(array[i]))
      return false;
  }

  return true;
};
