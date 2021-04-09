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

class WbTriangleMesh {
  constructor() {
    this.isValid = false;

    this.coordinates = [];
    this.coordIndices = [];
    this.tmpNormalIndices = [];
    this.tmpTexIndices = [];
    this.normals = [];
    this.textureCoordinates = [];
  }

  vertex(triangle, vertex, component) {
    return this.coordinates[this.coordinateIndex(triangle, vertex, component)];
  }

  coordinateIndex(triangle, vertex, component) {
    return 3 * this.coordIndices[this.index(triangle, vertex)] + component;
  }

  index(triangle, vertex) {
    return 3 * triangle + vertex;
  }

  normal(triangle, vertex, component) {
    return this.normals[3 * this.index(triangle, vertex) + component];
  }

  textureCoordinate(triangle, vertex, component) {
    return this.textureCoordinates[2 * this.index(triangle, vertex) + component];
  }

  init(coord, coordIndex, normal, normalIndex, texCoord, texCoordIndex, counterClockwise) {
    this.coordIndices = coordIndex;
    this.tmpTexIndices = texCoordIndex;
    this.tmpNormalIndices = normalIndex;
    this.numberOfTriangles = this.coordIndices.length / 3;

    if (!counterClockwise)
      this.reverseIndexOrder();

    this.finalPass(coord, normal, texCoord);

    // validity switch
    this.isValid = true;
  }

  // reverse the order of the second and third element
  // of each triplet of the this.coordIndices and this.tmpTexIndices arrays
  reverseIndexOrder() {
    const coordIndicesSize = this.coordIndices.length;
    if (coordIndicesSize % 3 !== 0)
      return;
    if (coordIndicesSize !== this.tmpTexIndices.length && this.tmpTexIndices.length !== 0)
      return;

    for (let i = 0; i < coordIndicesSize; i += 3) {
      const i1 = i + 1;
      const i2 = i + 2;
      const third = this.coordIndices[i2];
      this.coordIndices[i2] = this.coordIndices[i1];
      this.coordIndices[i1] = third;

      const thirdIndex = this.tmpTexIndices[i2];
      this.tmpTexIndices[i2] = this.tmpTexIndices[i1];
      this.tmpTexIndices[i1] = thirdIndex;
    }
  }

  // populate this.coordinates, this.textureCoordinates and this.normals
  finalPass(coord, normal, texCoord) {
    const texCoordSize = typeof texCoord !== 'undefined' ? texCoord.length : 0;
    const normalSize = typeof normal !== 'undefined' ? normal.length : 0;

    for (let i = 0; i < coord.length; ++i) {
      let vertex = coord[i];

      this.coordinates.push(vertex.x);
      this.coordinates.push(vertex.y);
      this.coordinates.push(vertex.z);
    }

    for (let t = 0; t < this.numberOfTriangles; ++t) { // foreach triangle
      const k = 3 * t;
      for (let v = 0; v < 3; ++v) { // foreach vertex
        const index = k + v;

        const indexNormal = this.tmpNormalIndices[index];
        if (indexNormal >= 0 && indexNormal < normalSize) {
          const nor = normal[indexNormal];
          this.normals.push(nor.x);
          this.normals.push(nor.y);
          this.normals.push(nor.z);
        }

        const indexTex = this.tmpTexIndices[index];
        if (indexTex >= 0 && indexTex < texCoordSize) {
          const tex = texCoord[indexTex];
          this.textureCoordinates.push(tex.x);
          this.textureCoordinates.push(1.0 - tex.y);
        } else {
          // foreach texture coordinate component
          this.textureCoordinates.push(0.5);
          this.textureCoordinates.push(0.5);
        }
      }
    }
  }
}

export {WbTriangleMesh};
