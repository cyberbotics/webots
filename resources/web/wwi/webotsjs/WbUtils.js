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

export {array3Pointer, arrayXPointer, direction, up, right, length}
