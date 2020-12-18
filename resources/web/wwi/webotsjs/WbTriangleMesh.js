// Copyright 1996-2020 Cyberbotics Ltd.
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

import {WbVector3} from "./utils/WbVector3.js";
import {WbTesselator} from "./utils/WbTesselator.js";

class WbTriangleMesh {
  constructor(){
    this.isValid = false;
    this.areTextureCoordinatesValid;
    this.normalsValid;

    this.numberOfTriangles;
    this.normalPerVertex;

    this.coordinates = [];
    this.coordIndices = [];
    this.tmpNormalIndices = [];
    this.tmpTexIndices = [];
    this.normals = []
    this.textureCoordinates = [];
    this.nonRecursiveTextureCoordinates = [];
    this.isNormalCreased = []

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


  init(coord, coordIndex, normal, normalIndex, texCoord, texCoordIndex, creaseAngle, counterClockwise, normalPerVertex) {
    this.normalPerVertex = normalPerVertex;

    // initial obvious check
    if (typeof coord === 'undefined' || coord.length === 0)
      return;
    if (typeof coordIndex === 'undefined' || coordIndex.length === 0)
      return;

    let nTrianglesEstimation = this.estimateNumberOfTriangles(coordIndex);  // overestimation of the number of triangles
    this.numberOfTriangles = 0;                                                         // keep the number of triangles


    // determine if the texture coordinate seems valid or not
    // this value will be used to determine the content of mTextureCoordinates
    const isTexCoordDefined = typeof texCoord !== 'undefined' && texCoord.length > 0;
    const isTexCoordIndexDefined = typeof texCoordIndex !== 'undefined' && texCoordIndex.length > 0;

    this.areTextureCoordinatesValid = isTexCoordDefined;
    if (isTexCoordDefined && isTexCoordIndexDefined && texCoordIndex.length !== coordIndex.length) {
      console.warn("Invalid texture mapping: the sizes of 'coordIndex' and 'texCoordIndex' mismatch. The default texture mapping is applied.");
      this.areTextureCoordinatesValid = false;
    }

    // determine if the normal seems valid or not
    // this value will be used to determine the content of mNormals
    const isNormalDefined = typeof normal !== 'undefined' && normal.length > 0;
    const isNormalIndexDefined = typeof normalIndex !== 'undefined' && normalIndex.length > 0;
    this.normalsValid = isNormalDefined;
    if (this.normalPerVertex) {
      if (isNormalDefined && isNormalIndexDefined && normalIndex.length != coordIndex.length) {
        console.warn("Invalid normal definition: the sizes of 'coordIndex' and 'normalIndex' mismatch. The normals will be computed using the creaseAngle.");
        this.normalsValid = false;
      }
    }

    // memory allocation of the tmp arrays (overestimated)
    const estimateSize = 3 * nTrianglesEstimation;
    this.coordIndices = [];
    const vertexSize = 3 * coord.length;
    if (this.areTextureCoordinatesValid)
      this.tmpTexIndices = [];
    this.tmpTriangleNormals = [];
    this.tmpVertexToTriangle = {};

    // passes to create the final arrays
    this.indicesPass(coord, coordIndex, (this.normalsValid && this.normalPerVertex && isNormalIndexDefined) ? normalIndex : coordIndex, (isTexCoordDefined && isTexCoordIndexDefined) ? texCoordIndex : coordIndex);
    this.numberOfTriangles = this.coordIndices.length / 3;
    if (this.normalsValid && !this.normalPerVertex && this.numberOfTriangles > normal.length) {
      console.warn("Invalid normal definition: the size of 'normal' should equal the number of triangles when 'normalPerVertex' is FALSE. The normals will be computed using the creaseAngle.");
      this.normalsValid = false;
    }

    if (!counterClockwise)
      this.reverseIndexOrder();
    let error = this.tmpNormalsPass(coord, normal);

    if (typeof error !== 'undefined')
      return;
    this.finalPass(coord, normal, texCoord, creaseAngle);

    // final obvious check
    if (this.numberOfTriangles <= 0) {
      console.error("The triangle mesh has no valid quad and no valid triangle.");
      return;
    }

    // validity switch
    this.isValid = true;
  }

  estimateNumberOfTriangles(coordIndex) {
    assert(coordIndex);

    let nTriangles = 0;
    let currentFaceIndicesCounter = 0;
    for(let i = 0; i < coordIndex.length; i++) {
      let index = coordIndex[i];
      if (index !== -1 && i === coordIndex.length - 1)
        ++currentFaceIndicesCounter;

      if (index === -1 || i === coordIndex.length - 1) {
        let nCurrentFaceTriangle = Math.max(0, currentFaceIndicesCounter - 2);
        nTriangles += nCurrentFaceTriangle;
        currentFaceIndicesCounter = 0;
      } else
        ++currentFaceIndicesCounter;
    }
    return nTriangles;
  }

  // populate this.coordIndices and this.tmpTexIndices with valid indices
  indicesPass(coord, coordIndex, normalIndex, texCoordIndex) {
    assert(!this.normalsValid || typeof normalIndex !== 'undefined');
    assert(!this.areTextureCoordinatesValid || typeof texCoordIndex !== 'undefined');
    assert(this.tmpNormalIndices.length === 0);
    assert(this.tmpTexIndices.length === 0);

    // parse coordIndex
    let currentFaceIndices = [];  // keep the coord, normal and tex indices of the current face
    const coordIndexSize = coordIndex.length;

    for (let i = 0; i < coordIndexSize; ++i) {
      // get the current index
      const index = coordIndex[i];

      // special case: last index not equal to -1
      // -> add a current index to the current face
      //    in order to have consistent data
      if (index !== -1 && i === coordIndexSize - 1)
        currentFaceIndices.push(new WbVector3(index, this.normalsValid ? normalIndex[i] : 0, this.areTextureCoordinatesValid ? texCoordIndex[i] : 0));
      const cfiSize = currentFaceIndices.length;
      // add the current face
      if (index === -1 || i === coordIndexSize - 1) {
        // check the validity of the current face
        // by checking if the range of the new face indices is valid
        let currentFaceValidity = true;
        for (let j = 0; j < cfiSize; ++j) {
          const cfi = currentFaceIndices[j].x;
          if (cfi < 0 || cfi >= coord.length)
            currentFaceValidity = false;
        }

        // add a face
        // -> tesselate everything in order to optimize dummy user input -> ex: [0 0 0 -1], [0 -1] or [0 1 2 0 -1])
        if (currentFaceValidity) {
          let tesselatorOutput = [];
          let tesselatorVectorInput = [];

          for (let j = 0; j < cfiSize; ++j) {
            const cfi = currentFaceIndices[j].x;
            tesselatorVectorInput.push(coord[cfi]);
          }

          WbTesselator.tesselate(currentFaceIndices, tesselatorVectorInput, tesselatorOutput);

          const toSize = tesselatorOutput.length;
          assert(toSize % 3 === 0);

          // we assume that GLU will give us back n-2 triangles for any polygon
          // it tesselates, so we can take shortcuts for triangles and quads
          // simplest case: polygon is triangle
          if (toSize === 3) {
            this.coordIndices.push(tesselatorOutput[0].x);
            this.coordIndices.push(tesselatorOutput[1].x);
            this.coordIndices.push(tesselatorOutput[2].x);

            if (this.normalsValid) {
              this.tmpNormalIndices.push(tesselatorOutput[0].y);
              this.tmpNormalIndices.push(tesselatorOutput[1].y);
              this.tmpNormalIndices.push(tesselatorOutput[2].y);
            }

            if (this.areTextureCoordinatesValid) {
              this.tmpTexIndices.push(tesselatorOutput[0].z);
              this.tmpTexIndices.push(tesselatorOutput[1].z);
              this.tmpTexIndices.push(tesselatorOutput[2].z);
            }
          }
          // polygon is quad (two triangles)
          else if (toSize == 6) {
            this.coordIndices.push(tesselatorOutput[0].x);
            this.coordIndices.push(tesselatorOutput[1].x);
            this.coordIndices.push(tesselatorOutput[2].x);
            this.coordIndices.push(tesselatorOutput[3].x);
            this.coordIndices.push(tesselatorOutput[4].x);
            this.coordIndices.push(tesselatorOutput[5].x);

            if (this.normalsValid) {
              this.tmpNormalIndices.push(tesselatorOutput[0].y);
              this.tmpNormalIndices.push(tesselatorOutput[1].y);
              this.tmpNormalIndices.push(tesselatorOutput[2].y);
              this.tmpNormalIndices.push(tesselatorOutput[3].y);
              this.tmpNormalIndices.push(tesselatorOutput[4].y);
              this.tmpNormalIndices.push(tesselatorOutput[5].y);
            }

            if (this.areTextureCoordinatesValid) {
              this.tmpTexIndices.push(tesselatorOutput[0].z);
              this.tmpTexIndices.push(tesselatorOutput[1].z);
              this.tmpTexIndices.push(tesselatorOutput[2].z);
              this.tmpTexIndices.push(tesselatorOutput[3].z);
              this.tmpTexIndices.push(tesselatorOutput[4].z);
              this.tmpTexIndices.push(tesselatorOutput[5].z);
            }
          }
          // 5+ vertex polygon
          // sometimes GLU will perform bizzarre tesselations of some polygons
          // that generate triangles with zero area. We remove these triangles
          // as they cause numerical errors, but removing them can cause
          // flickering holes to appear in the polygon. We need to stitch the gap
          // left by the now-missing triangle back together to re-close the polygon
          else {
            for (let j = 0; j < toSize; j += 3) {
              const a = coord[tesselatorOutput[j].x];
              const b = coord[tesselatorOutput[j + 1].x];
              const c = coord[tesselatorOutput[j + 2].x];

              // check for colinear edges, and discard this triangle if found
              const d = b.sub(a);
              const e = c.sub(b);
              // don't append if edges are co-linear
              if (d.cross(e).almostEquals(WbVector3(), 0.00001))
                continue;
              // don't append if two vertices are on the same spot
              if (a.equal(b) || a.equal(c) || b.equal(c)) {
                console.warn("Duplicate vertices detected while triangulating mesh. Try opening your model in 3D modeling software and removing duplicate vertices, then re-importing.");
                continue;
              }
              // see if this triangle has any overlapping vertices and snip triangle to improve tesselation and fill holes
              const snippedIndices = cutTriangleIfNeeded(coord, tesselatorOutput, j);
              assert(snippedIndices.length % 3 === 0);
              for (let k = 0; k < snippedIndices.length; k += 3) {
                this.coordIndices.push(snippedIndices[k].x);
                this.coordIndices.push(snippedIndices[k + 1].x);
                this.coordIndices.push(snippedIndices[k + 2].x);
                if (this.normalsValid) {
                  this.tmpNormalIndices.push(snippedIndices[k].y);
                  this.tmpNormalIndices.push(snippedIndices[k + 1].y);
                  this.tmpNormalIndices.push(snippedIndices[k + 2].y);
                }
                if (this.areTextureCoordinatesValid) {
                  this.tmpTexIndices.push(snippedIndices[k].z);
                  this.tmpTexIndices.push(snippedIndices[k + 1].z);
                  this.tmpTexIndices.push(snippedIndices[k + 2].z);
                }
              }
            }
          }
        }
        // warning: the current face is invalid
        else {
          console.warn("current face is invalid");
        }

        currentFaceIndices = [];
      }
      // add a coordIndex to the currentFace
      else
        currentFaceIndices.push(new WbVector3(index, this.normalsValid ? normalIndex[i] : 0, this.areTextureCoordinatesValid ? texCoordIndex[i] : 0));
    }

    assert(this.coordIndices.length === this.tmpNormalIndices.length || this.tmpNormalIndices.length === 0);
    assert(this.coordIndices.length === this.tmpTexIndices.length || this.tmpTexIndices.length === 0);
    assert(this.coordIndices.length % 3 === 0);
  }

  // reverse the order of the second and third element
  // of each triplet of the this.coordIndices and this.tmpTexIndices arrays
  reverseIndexOrder() {
    const coordIndicesSize = this.coordIndices.length;
    assert(coordIndicesSize % 3 === 0);
    assert(coordIndicesSize === this.tmpTexIndices.length || this.tmpTexIndices.length === 0);

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

  // populate this.tmpTriangleNormals from coord and this.coordIndices
  tmpNormalsPass(coord, normal) {
    assert(this.numberOfTriangles === this.coordIndices.length / 3);
    assert(this.coordIndices.length % 3 === 0);

    if (this.normalsValid && this.normalPerVertex)
      return undefined;  // normal are already defined per vertex

    // 1. compute normals per triangle
    for (let i = 0; i < this.numberOfTriangles; ++i) {
      if (this.normalsValid)
        this.tmpTriangleNormals.push(normal[i].normalized());
      else {
        const j = 3 * i;
        const indexA = this.coordIndices[j];
        const indexB = this.coordIndices[j + 1];
        const indexC = this.coordIndices[j + 2];

        assert(indexA >= 0 && indexA < coord.length);
        assert(indexB >= 0 && indexB < coord.length);
        assert(indexC >= 0 && indexC < coord.length);

        const posA = coord[indexA];
        const posB = coord[indexB];
        const posC = coord[indexC];

        const v1 = posB.sub(posA);
        const v2 = posC.sub(posA);
        let n = v1.cross(v2);
        const length = n.length();
        if (length === 0.0)
          return undefined;

        n.div(length);
        this.tmpTriangleNormals.push(n);
      }
    }

    assert(this.tmpTriangleNormals.length === this.numberOfTriangles);
    // 2. compute the map coordIndex->triangleIndex
    for (let t = 0; t < this.numberOfTriangles; ++t) {
      const k = 3 * t;

      let index = this.coordIndices[k];

      if(typeof this.tmpVertexToTriangle[index] === 'undefined')
        this.tmpVertexToTriangle[index] = [];
      this.tmpVertexToTriangle[index].push(t);

      index = this.coordIndices[k + 1];
      if(typeof this.tmpVertexToTriangle[index] === 'undefined')
        this.tmpVertexToTriangle[index] = [];
      this.tmpVertexToTriangle[index].push(t);

      index = this.coordIndices[k + 2];
      if(typeof this.tmpVertexToTriangle[index] === 'undefined')
        this.tmpVertexToTriangle[index] = [];
      this.tmpVertexToTriangle[index].push(t);
    }

    return;
  }

  // populate this.coordinates, this.textureCoordinates and this.normals
  finalPass(coord, normal, texCoord, creaseAngle) {
    assert(coord && coord.length > 0);
    assert(this.tmpTriangleNormals.length === this.numberOfTriangles || (this.normalsValid && this.normalPerVertex));
    assert(this.numberOfTriangles === this.coordIndices.length / 3);
    assert(this.coordIndices.length % 3 === 0);
    assert(this.coordIndices.length === this.tmpTexIndices.length || this.tmpTexIndices.length === 0);
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

    for (let t = 0; t < this.numberOfTriangles; ++t) {  // foreach triangle
      const k = 3 * t;
      for (let v = 0; v < 3; ++v) {  // foreach vertex
        const index = k + v;
        const indexCoord = this.coordIndices[index];

        // compute the normal per vertex (from normal per triangle)
        if (!this.normalsValid || !this.normalPerVertex) {
          let triangleNormal = new WbVector3();
          const faceNormal = this.tmpTriangleNormals[t];
          const linkedTriangles = this.tmpVertexToTriangle[indexCoord];
          const ltSize = linkedTriangles.length;

          // stores the normals of the linked triangles which are already used.
          const linkedTriangleNormals = [];
          let creasedLinkedTriangleNumber = 0;
          let linkedTriangleNormalsIndex = 0;

          for (let i = 0; i < ltSize; ++i) {
            const linkedTriangleIndex = linkedTriangles[i];
            if (linkedTriangleIndex >= 0 && linkedTriangleIndex < this.numberOfTriangles) {
              const linkedTriangleNormal = this.tmpTriangleNormals[linkedTriangleIndex];
              // perform the creaseAngle check
              if (faceNormal.angle(linkedTriangleNormal) < creaseAngle) {
                creasedLinkedTriangleNumber++;
                let found = false;
                // we don't want coplanar face normals on e.g. a cylinder to bias a
                // normal and cause discontinuities, so don't include duplicated
                // normals in the smoothing pass
                for (let lN = 0; lN < linkedTriangleNormalsIndex; ++lN) {
                  const currentLinkedTriangleNormal = linkedTriangleNormals[lN];
                  if (currentLinkedTriangleNormal.almostEquals(linkedTriangleNormal, 0.0001)) {
                    found = true;
                    break;
                  }
                }
                if (!found) {
                  triangleNormal = triangleNormal.add(linkedTriangleNormal);
                  linkedTriangleNormals[linkedTriangleNormalsIndex] = linkedTriangleNormal;
                  linkedTriangleNormalsIndex++;
                }
              }
            }
          }

          if (triangleNormal.isNull())
            triangleNormal = faceNormal;
          else
            triangleNormal.normalize();

          // populate the remaining two final arrays
          this.normals.push(triangleNormal.x);
          this.normals.push(triangleNormal.y);
          this.normals.push(triangleNormal.z);
          this.isNormalCreased.push(creasedLinkedTriangleNumber === ltSize);
        } else {  // normal already defined per vertex
          const indexNormal = this.tmpNormalIndices[index];
          if (indexNormal >= 0 && indexNormal < normalSize) {
            const nor = normal[indexNormal];
            this.normals.push(nor.x);
            this.normals.push(nor.y);
            this.normals.push(nor.z);
            this.isNormalCreased.push(false);
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

    if (!this.areTextureCoordinatesValid)
      this.setDefaultTextureCoordinates(coord);

    // check the resulted size
    assert(this.coordinates.length === 3 * coordSize);
    assert(this.normals.length === 3 * 3 * this.numberOfTriangles);
    assert(this.isNormalCreased.length === 3 * this.numberOfTriangles);
    assert(this.textureCoordinates.length === 0 || this.textureCoordinates.length === 2 * 3 * this.numberOfTriangles);
    assert(this.nonRecursiveTextureCoordinates.length === 0 || this.nonRecursiveTextureCoordinates.length === 2 * 3 * this.numberOfTriangles);
  }

  setDefaultTextureCoordinates(coord) {
    const minBound = new WbVector3(this.min[0], this.min[1], this.min[2]);
    const maxBound = new WbVector3(this.max[0], this.max[1], this.max[2]);
    const size = maxBound.sub(minBound);

    // compute size and find longest and second-longest dimensions for default mapping
    let longestDimension = -1;
    let secondLongestDimension = -1;
    for (let i = 0; i < 3; ++i) {
      if (longestDimension < 0 || size[i] > size[longestDimension]) {
        secondLongestDimension = longestDimension;
        longestDimension = i;
      } else if (secondLongestDimension < 0 || size[i] > size[secondLongestDimension])
        secondLongestDimension = i;
    }

    assert(longestDimension >= 0 && secondLongestDimension >= 0);

    let index = 0;
    let vertices = [];
    for (let t = 0; t < this.numberOfTriangles; ++t) {  // foreach triangle
      vertices[0] = coord[this.coordIndices[index]];
      vertices[1] = coord[this.coordIndices[index + 1]];
      vertices[2] = coord[this.coordIndices[index + 2]];

      // compute face center and normal
      const edge1 = vertices[1].sub(vertices[0]);
      const edge2 = vertices[2].sub(vertices[0]);
      let normal = edge1.cross(edge2);
      normal.normalize();
      const origin = new WbVector3(vertices[0].add(vertices[1]).add(vertices[2]).div(3.0));

      // compute intersection with the bounding box
      const faceNormal = new WbRay(origin, normal);
      let tmin, tmax;
      const result = faceNormal.intersects(minBound, maxBound, tmin, tmax);
      assert(result[0]);
      const faceIndex = WbBox.findIntersectedFace(minBound, maxBound, origin.add(normal.mul(result[1])));

      for (let v = 0; v < 3; ++v) {  // foreach vertex
        // compute default texture mapping
        this.textureCoordinates.push((vertices[v].get(longestDimension) - this.min.get(longestDimension)) / size.get(longestDimension));
        this.textureCoordinates.push(1.0 - (vertices[v].get(secondLongestDimension) - this.min.get(secondLongestDimension)) / size.get(longestDimension));

        // compute non-recursive mapping
        const uv = WbBox.computeTextureCoordinate(minBound, maxBound, vertices[v], true, faceIndex);
        this.nonRecursiveTextureCoordinates.push(uv.x);
        this.nonRecursiveTextureCoordinates.push(uv.y);
      }

      index += 3;
    }
  }
}

export {WbTriangleMesh}
