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

import {WbVector3} from './utils/wbVector3.js';

class WbTriangleMesh {
  constructor() {
    this.isValid = false;

    this.numberOfTriangles = undefined;
    this.normalPerVertex = undefined;

    this.coordinates = [];
    this.coordIndices = [];
    this.tmpNormalIndices = [];
    this.tmpTexIndices = [];
    this.normals = [];
    this.textureCoordinates = [];
    this.nonRecursiveTextureCoordinates = [];

    this.min = [Number.MAX_VALUE, Number.MAX_VALUE, Number.MAX_VALUE];
    this.max = [Number.MIN_VALUE, Number.MIN_VALUE, Number.MIN_VALUE];
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

  nonRecursiveTextureCoordinate(triangle, vertex, component) {
    return this.nonRecursiveTextureCoordinates[2 * this.index(triangle, vertex) + component];
  }

  init(coord, coordIndex, normal, normalIndex, texCoord, texCoordIndex, counterClockwise, normalPerVertex) {
    this.normalPerVertex = normalPerVertex;

    // initial obvious check
    if (typeof coord === 'undefined' || coord.length === 0)
      return;
    if (typeof coordIndex === 'undefined' || coordIndex.length === 0)
      return;

    // determine if the texture coordinate seems valid or not
    const isTexCoordDefined = typeof texCoord !== 'undefined' && texCoord.length > 0;
    const isTexCoordIndexDefined = typeof texCoordIndex !== 'undefined' && texCoordIndex.length > 0;

    this.areTextureCoordinatesValid = isTexCoordDefined;
    if (isTexCoordDefined && isTexCoordIndexDefined && texCoordIndex.length !== coordIndex.length) {
      console.warn("Invalid texture mapping: the sizes of 'coordIndex' and 'texCoordIndex' mismatch. The default texture mapping is applied.");
      this.areTextureCoordinatesValid = false;
    }

    // determine if the normal seems valid or not
    const isNormalDefined = typeof normal !== 'undefined' && normal.length > 0;
    const isNormalIndexDefined = typeof normalIndex !== 'undefined' && normalIndex.length > 0;
    this.normalsValid = isNormalDefined;
    if (this.normalPerVertex) {
      if (isNormalDefined && isNormalIndexDefined && normalIndex.length !== coordIndex.length) {
        console.warn("Invalid normal definition: the sizes of 'coordIndex' and 'normalIndex' mismatch.");
        this.normalsValid = false;
      }
    }

    this.coordIndices = coordIndex;
    this.tmpTexIndices = texCoordIndex;
    this.tmpNormalIndices = normalIndex;
    this.numberOfTriangles = this.coordIndices.length / 3;

    if (this.normalsValid && !this.normalPerVertex && this.numberOfTriangles > normal.length) {
      console.warn("Invalid normal definition: the size of 'normal' should equal the number of triangles when 'normalPerVertex' is FALSE.");
      this.normalsValid = false;
    }

    if (!counterClockwise)
      this.reverseIndexOrder();

    this.finalPass(coord, normal, texCoord);

    // final obvious check
    if (this.numberOfTriangles <= 0) {
      console.error('The triangle mesh has no valid quad and no valid triangle.');
      return;
    }

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

      if (this.areTextureCoordinatesValid) {
        const thirdIndex = this.tmpTexIndices[i2];
        this.tmpTexIndices[i2] = this.tmpTexIndices[i1];
        this.tmpTexIndices[i1] = thirdIndex;
      }
    }
  }

  // populate this.coordinates, this.textureCoordinates and this.normals
  finalPass(coord, normal, texCoord) {
    const texCoordSize = typeof texCoord !== 'undefined' ? texCoord.length : 0;
    const normalSize = typeof normal !== 'undefined' ? normal.length : 0;
    const coordSize = coord.length;

    // populate the vertex array
    let vertex = coord[0];
    this.max[0] = vertex.x;
    this.max[1] = vertex.y;
    this.max[2] = vertex.z;
    this.min[0] = this.max[0];
    this.min[1] = this.max[1];
    this.min[2] = this.max[2];
    for (let i = 0; i < coordSize; ++i) {
      vertex = coord[i];

      const x = vertex.x;
      if (this.max[0] < x)
        this.max[0] = x;
      else if (this.min[0] > x)
        this.min[0] = x;

      const y = vertex.y;
      if (this.max[1] < y)
        this.max[1] = y;
      else if (this.min[1] > y)
        this.min[1] = y;

      const z = vertex.z;
      if (this.max[2] < z)
        this.max[2] = z;
      else if (this.min[2] > z)
        this.min[2] = z;

      this.coordinates.push(x);
      this.coordinates.push(y);
      this.coordinates.push(z);
    }

    for (let t = 0; t < this.numberOfTriangles; ++t) { // foreach triangle
      const k = 3 * t;
      for (let v = 0; v < 3; ++v) { // foreach vertex
        const index = k + v;

        // compute the normal per vertex (from normal per triangle)
        if (this.normalsValid && this.normalPerVertex) {
          const indexNormal = this.tmpNormalIndices[index];
          if (indexNormal >= 0 && indexNormal < normalSize) {
            const nor = normal[indexNormal];
            this.normals.push(nor.x);
            this.normals.push(nor.y);
            this.normals.push(nor.z);
          }
        }

        if (this.areTextureCoordinatesValid) {
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
}

export {WbTriangleMesh};
