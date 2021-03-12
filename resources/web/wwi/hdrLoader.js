// Copyright 1996-2021 Cyberbotics Ltd.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

function loadHdr(url, onLoad) {
  fetch(url)
    .then(response => response.arrayBuffer())
    .then(function(data) {
      onLoad(parse(data));
    });
}

function parse(buffer) {
  let dec = new TextDecoder('utf-8');
  let dat = dec.decode(buffer);
  const lines = dat.split('\n');
  const dimension = lines[3].split(' ');
  let width = parseInt(dimension[1]);
  let height = parseInt(dimension[3]);

  const byteArray = new Uint8Array(buffer);
  byteArray.pos = lines[0].length + lines[1].length + lines[2].length + lines[3].length + 4;

  const imageRgbaData = new Uint8Array(byteArray.subarray(byteArray.pos));
  let data;
  // adapted from http://www.graphics.cornell.edu/~bjw/rgbe.html
  const RGBEByteToRGBFloat = function(sourceArray, sourceOffset, destArray, destOffset) {
    const e = sourceArray[sourceOffset + 3];
    const scale = Math.pow(2.0, e - 128.0) / 255.0;
    destArray[destOffset + 0] = sourceArray[sourceOffset + 0] * scale;
    destArray[destOffset + 1] = sourceArray[sourceOffset + 1] * scale;
    destArray[destOffset + 2] = sourceArray[sourceOffset + 2] * scale;
  };
  const numElements = (imageRgbaData.length / 4) * 3;
  const floatArray = new Float32Array(numElements);
  for (let j = 0; j < numElements; j++)
    RGBEByteToRGBFloat(imageRgbaData, j * 4, floatArray, j * 3);

  data = floatArray;

  return {
    width: width,
    height: height,
    data: data
  };
}

export { loadHdr };
