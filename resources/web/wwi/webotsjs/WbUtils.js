import {M_PI} from "./WbConstants.js"


function array3Pointer(x, y, z) {
  let data = new Float32Array([x, y, z]);
  let nDataBytes = data.length * data.BYTES_PER_ELEMENT;
  let dataPtr = Module._malloc(nDataBytes);
  let dataHeap = new Uint8Array(Module.HEAPU8.buffer, dataPtr, nDataBytes);
  dataHeap.set(new Uint8Array(data.buffer));

  return dataHeap.byteOffset;
}

function arrayXPointer(array) {
  let data = new Uint8ClampedArray(array);
  let nDataBytes = data.length * data.BYTES_PER_ELEMENT;
  let dataPtr = Module._malloc(nDataBytes);
  let dataHeap = new Uint8Array(Module.HEAPU8.buffer, dataPtr, nDataBytes);
  dataHeap.set(new Uint8Array(data.buffer));

  return dataHeap.byteOffset;
}

function direction(vec4)  {
  let c = Math.cos(vec4.w), s = Math.sin(vec4.w), t = 1 - c;
  let tTimesZ = t * vec4.z;
  return glm.vec3(tTimesZ * vec4.x + s * vec4.y, tTimesZ * vec4.y - s * vec4.x, tTimesZ * vec4.z + c);
}

function right(vec4) {
  let c = Math.cos(vec4.w), s = Math.sin(vec4.w), t = 1 - c;
  let tTimesX = t * vec4.x;
  return glm.vec3(tTimesX * vec4.x + c, tTimesX * vec4.w + s * vec4.z, tTimesX * vec4.z - s * vec4.y);
}

function up(vec4) {
  let c = Math.cos(vec4.w), s = Math.sin(vec4.w), t = 1 - c;
  let tTimesY = t * vec4.y;
  return glm.vec3(tTimesY * vec4.x - s * vec4.z, tTimesY * vec4.y + c, tTimesY * vec4.z + s * vec4.x);
}

function length(vec3) {
  return Math.sqrt(vec3.x * vec3.x + vec3.y * vec3.y + vec3.z * vec3.z)
}

function vec4ToQuaternion(vec4) {
  let halfAngle = 0.5 * vec4.w;
  let sinusHalfAngle = Math.sin(halfAngle), cosinusHalfAngle = Math.cos(halfAngle);
  return glm.quat(cosinusHalfAngle, vec4.x * sinusHalfAngle, vec4.y * sinusHalfAngle, vec4.z * sinusHalfAngle);
}

function quaternionToVec4(quat) {
  let angle;
  if (quat.w >= 1.0)
    angle = 0.0;
  else if (quat.w <= -1.0)
    angle = 2.0 * M_PI;
  else
    angle = 2.0 * Math.acos(quat.w);

  // normalise axes
  let inv = 1.0 / Math.sqrt(quat.x * quat.x + quat.y * quat.y + quat.z* quat.z);
  let x = quat.x * inv;
  let y = quat.y * inv;
  let z = quat.z * inv;

  return glm.vec4(x, y, z, angle)
}

function fromAxisAngle(x, y, z, angle) {
  let result = glm.vec4();
  let l = x * x + y * y + z * z;
  if (l > 0.0) {
    angle *= 0.5;
    result.w = Math.cos(angle);
    l = Math.sin(angle) / Math.sqrt(l);
    result.x = x * l;
    result.y = y * l;
    result.z = z * l;
  } else {
    result.w = 1.0;
    result.x = 0.0;
    result.y = 0.0;
    result.z = 0.0;
  }
  return result
}


export {array3Pointer, arrayXPointer, direction, up, right, length, vec4ToQuaternion, quaternionToVec4, fromAxisAngle}
