import WbBox from '../WbBox.js';
import WbRay from './WbRay.js';
import WbTesselator from '../../wren/WbTesselator.js';
import WbVector3 from './WbVector3.js';

export default class WbTriangleMesh {
  #normalsValid;
  #coordinates;
  #coordIndices;
  #max;
  #min;
  #nonRecursiveTextureCoordinates;
  #normals;
  #normalPerVertex;
  #textureCoordinates;
  #tmpNormalIndices;
  #tmpTexIndices;
  #tmpTriangleNormals;
  #tmpVertexToTriangle;
  constructor() {
    this.isValid = false;
    this.areTextureCoordinatesValid = false;
    this.#normalsValid = false;

    this.numberOfTriangles = undefined;
    this.#normalPerVertex = undefined;

    this.#coordinates = [];
    this.#coordIndices = [];
    this.#tmpNormalIndices = [];
    this.#tmpTexIndices = [];
    this.#normals = [];
    this.#textureCoordinates = [];
    this.#nonRecursiveTextureCoordinates = [];

    this.#min = [Number.MAX_VALUE, Number.MAX_VALUE, Number.MAX_VALUE];
    this.#max = [Number.MIN_VALUE, Number.MIN_VALUE, Number.MIN_VALUE];
  }

  vertex(triangle, vertex, component) {
    return this.#coordinates[this.#coordinateIndex(triangle, vertex, component)];
  }

  #coordinateIndex(triangle, vertex, component) {
    return 3 * this.#coordIndices[this.index(triangle, vertex)] + component;
  }

  index(triangle, vertex) {
    return 3 * triangle + vertex;
  }

  normal(triangle, vertex, component) {
    return this.#normals[3 * this.index(triangle, vertex) + component];
  }

  textureCoordinate(triangle, vertex, component) {
    return this.#textureCoordinates[2 * this.index(triangle, vertex) + component];
  }

  nonRecursiveTextureCoordinate(triangle, vertex, component) {
    return this.#nonRecursiveTextureCoordinates[2 * this.index(triangle, vertex) + component];
  }

  init(coord, coordIndex, normal, normalIndex, texCoord, texCoordIndex, creaseAngle, normalPerVertex) {
    this.#normalPerVertex = normalPerVertex;

    // initial obvious check
    if (typeof coord === 'undefined' || coord.length === 0)
      return;
    if (typeof coordIndex === 'undefined' || coordIndex.length === 0)
      return;

    this.numberOfTriangles = 0; // keep the number of triangles

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
    this.#normalsValid = isNormalDefined;
    if (this.#normalPerVertex) {
      if (isNormalDefined && isNormalIndexDefined && normalIndex.length !== coordIndex.length) {
        console.warn("Invalid normal definition: the sizes of 'coordIndex' and 'normalIndex' mismatch. The normals will be computed using the creaseAngle.");
        this.#normalsValid = false;
      }
    }

    // memory allocation of the tmp arrays (overestimated)
    this.#coordIndices = [];
    if (this.areTextureCoordinatesValid)
      this.#tmpTexIndices = [];
    this.#tmpTriangleNormals = [];
    this.#tmpVertexToTriangle = {};

    // passes to create the final arrays
    this.#indicesPass(coord, coordIndex, (this.#normalsValid && this.#normalPerVertex && isNormalIndexDefined)
      ? normalIndex : coordIndex, (isTexCoordDefined && isTexCoordIndexDefined) ? texCoordIndex : coordIndex);
    this.numberOfTriangles = this.#coordIndices.length / 3;
    if (this.#normalsValid && !this.#normalPerVertex && this.numberOfTriangles > normal.length) {
      console.warn("Invalid normal definition: the size of 'normal' should equal the number of triangles when 'normalPerVertex' is FALSE. The normals will be computed using the creaseAngle.");
      this.#normalsValid = false;
    }

    const error = this.#tmpNormalsPass(coord, normal);

    if (typeof error !== 'undefined')
      return;
    this.#finalPass(coord, normal, texCoord, creaseAngle);

    // final obvious check
    if (this.numberOfTriangles <= 0) {
      console.error('The triangle mesh has no valid quad and no valid triangle.');
      return;
    }

    // validity switch
    this.isValid = true;
  }

  initMesh(coord, normal, texCoord, index) {
    this.numberOfTriangles = index.length / 3;

    this.#coordIndices = [...index];
    this.#coordinates = [...coord];

    for (let t = 0; t < this.numberOfTriangles; ++t) { // foreach triangle
      for (let v = 0; v < 3; ++v) { // foreach vertex
        const currentIndex = this.#coordIndices[3 * t + v];
        this.#textureCoordinates.push(texCoord[2 * currentIndex]);
        this.#textureCoordinates.push(texCoord[2 * currentIndex + 1]);
        this.#normals.push(normal[3 * currentIndex]);
        this.#normals.push(normal[3 * currentIndex + 1]);
        this.#normals.push(normal[3 * currentIndex + 2]);
      }
    }

    // validity switch
    this.isValid = true;
  }

  // populate this.#coordIndices and this.#tmpTexIndices with valid indices
  #indicesPass(coord, coordIndex, normalIndex, texCoordIndex) {
    if (this.#normalsValid && typeof normalIndex === 'undefined')
      return;
    if (this.areTextureCoordinatesValid && typeof texCoordIndex === 'undefined')
      return;
    if (this.#tmpNormalIndices.length !== 0 || this.#tmpTexIndices.length !== 0)
      return;

    // parse coordIndex
    let currentFaceIndices = []; // keep the coord, normal and tex indices of the current face
    const coordIndexSize = coordIndex.length;

    for (let i = 0; i < coordIndexSize; ++i) {
      // get the current index
      const index = coordIndex[i];

      // special case: last index not equal to -1
      // -> add a current index to the current face
      //    in order to have consistent data
      if (index !== -1 && i === coordIndexSize - 1)
        currentFaceIndices.push(new WbVector3(index, this.#normalsValid ? normalIndex[i] : 0,
          this.areTextureCoordinatesValid ? texCoordIndex[i] : 0));
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
          const tesselatorOutput = [];
          const tesselatorVectorInput = [];
          for (let j = 0; j < cfiSize; ++j) {
            const cfi = currentFaceIndices[j].x;
            tesselatorVectorInput.push(coord[cfi]);
          }

          WbTesselator.tesselate(currentFaceIndices, tesselatorVectorInput, tesselatorOutput);

          const toSize = tesselatorOutput.length;
          if (toSize % 3 !== 0)
            return;

          // we assume that GLU will give us back n-2 triangles for any polygon
          // it tesselates, so we can take shortcuts for triangles and quads
          // simplest case: polygon is triangle
          if (toSize === 3) {
            this.#coordIndices.push(tesselatorOutput[0].x);
            this.#coordIndices.push(tesselatorOutput[1].x);
            this.#coordIndices.push(tesselatorOutput[2].x);

            if (this.#normalsValid) {
              this.#tmpNormalIndices.push(tesselatorOutput[0].y);
              this.#tmpNormalIndices.push(tesselatorOutput[1].y);
              this.#tmpNormalIndices.push(tesselatorOutput[2].y);
            }

            if (this.areTextureCoordinatesValid) {
              this.#tmpTexIndices.push(tesselatorOutput[0].z);
              this.#tmpTexIndices.push(tesselatorOutput[1].z);
              this.#tmpTexIndices.push(tesselatorOutput[2].z);
            }
          } else if (toSize === 6) { // polygon is quad (two triangles)
            this.#coordIndices.push(tesselatorOutput[0].x);
            this.#coordIndices.push(tesselatorOutput[1].x);
            this.#coordIndices.push(tesselatorOutput[2].x);
            this.#coordIndices.push(tesselatorOutput[3].x);
            this.#coordIndices.push(tesselatorOutput[4].x);
            this.#coordIndices.push(tesselatorOutput[5].x);

            if (this.#normalsValid) {
              this.#tmpNormalIndices.push(tesselatorOutput[0].y);
              this.#tmpNormalIndices.push(tesselatorOutput[1].y);
              this.#tmpNormalIndices.push(tesselatorOutput[2].y);
              this.#tmpNormalIndices.push(tesselatorOutput[3].y);
              this.#tmpNormalIndices.push(tesselatorOutput[4].y);
              this.#tmpNormalIndices.push(tesselatorOutput[5].y);
            }

            if (this.areTextureCoordinatesValid) {
              this.#tmpTexIndices.push(tesselatorOutput[0].z);
              this.#tmpTexIndices.push(tesselatorOutput[1].z);
              this.#tmpTexIndices.push(tesselatorOutput[2].z);
              this.#tmpTexIndices.push(tesselatorOutput[3].z);
              this.#tmpTexIndices.push(tesselatorOutput[4].z);
              this.#tmpTexIndices.push(tesselatorOutput[5].z);
            }
          } else { // 5+ vertex polygon
            for (let j = 0; j < toSize; j += 3) {
              const a = coord[tesselatorOutput[j].x];
              const b = coord[tesselatorOutput[j + 1].x];
              const c = coord[tesselatorOutput[j + 2].x];

              // check for colinear edges, and discard this triangle if found
              const d = b.sub(a);
              const e = c.sub(b);
              // don't append if edges are co-linear
              if (d.cross(e).almostEquals(new WbVector3(), 0.00001))
                continue;

              // don't append if two vertices are on the same spot
              if (a.equal(b) || a.equal(c) || b.equal(c)) {
                console.warn('Duplicate vertices detected while triangulating mesh. ' +
                  'Try opening your model in 3D modeling software and removing duplicate vertices, then re-importing.');
                continue;
              }

              // see if this triangle has any overlapping vertices and snip triangle to improve tesselation and fill holes
              const snippedIndices = this.#cutTriangleIfNeeded(coord, tesselatorOutput, j);
              console.assert(snippedIndices.length % 3 === 0);
              for (let k = 0; k < snippedIndices.length; k += 3) {
                this.#coordIndices.push(snippedIndices[k].x);
                this.#coordIndices.push(snippedIndices[k + 1].x);
                this.#coordIndices.push(snippedIndices[k + 2].x);
                if (this.#normalsValid) {
                  this.#tmpNormalIndices.push(snippedIndices[k].y);
                  this.#tmpNormalIndices.push(snippedIndices[k + 1].y);
                  this.#tmpNormalIndices.push(snippedIndices[k + 2].y);
                }
                if (this.areTextureCoordinatesValid) {
                  this.#tmpTexIndices.push(snippedIndices[k].z);
                  this.#tmpTexIndices.push(snippedIndices[k + 1].z);
                  this.#tmpTexIndices.push(snippedIndices[k + 2].z);
                }
              }
            }
          }
        } else
          console.warn('current face is invalid');

        currentFaceIndices = [];
      } else // add a coordIndex to the currentFace
        currentFaceIndices.push(new WbVector3(index, this.#normalsValid ? normalIndex[i] : 0,
          this.areTextureCoordinatesValid ? texCoordIndex[i] : 0));
    }
  }

  // populate this.#tmpTriangleNormals from coord and this.#coordIndices
  #tmpNormalsPass(coord, normal) {
    if (this.numberOfTriangles !== this.#coordIndices.length / 3 || this.#coordIndices.length % 3 !== 0)
      return;

    if (this.#normalsValid && this.#normalPerVertex)
      return undefined; // normal are already defined per vertex

    // 1. compute normals per triangle
    for (let i = 0; i < this.numberOfTriangles; ++i) {
      if (this.#normalsValid)
        this.#tmpTriangleNormals.push(normal[i].normalized());
      else {
        const j = 3 * i;
        const indexA = this.#coordIndices[j];
        const indexB = this.#coordIndices[j + 1];
        const indexC = this.#coordIndices[j + 2];

        const posA = coord[indexA];
        const posB = coord[indexB];
        const posC = coord[indexC];

        const v1 = posB.sub(posA);
        const v2 = posC.sub(posA);
        let n = v1.cross(v2);
        const length = n.length();
        if (length === 0.0)
          return undefined;

        n = n.div(length);
        this.#tmpTriangleNormals.push(n);
      }
    }

    // 2. compute the map coordIndex->triangleIndex
    for (let t = 0; t < this.numberOfTriangles; ++t) {
      const k = 3 * t;

      let index = this.#coordIndices[k];

      if (typeof this.#tmpVertexToTriangle[index] === 'undefined')
        this.#tmpVertexToTriangle[index] = [];
      this.#tmpVertexToTriangle[index].push(t);

      index = this.#coordIndices[k + 1];
      if (typeof this.#tmpVertexToTriangle[index] === 'undefined')
        this.#tmpVertexToTriangle[index] = [];
      this.#tmpVertexToTriangle[index].push(t);

      index = this.#coordIndices[k + 2];
      if (typeof this.#tmpVertexToTriangle[index] === 'undefined')
        this.#tmpVertexToTriangle[index] = [];
      this.#tmpVertexToTriangle[index].push(t);
    }
  }

  // populate this.#coordinates, this.#textureCoordinates and this.#normals
  #finalPass(coord, normal, texCoord, creaseAngle) {
    if (typeof coord === 'undefined' || coord.length <= 0)
      return;
    if (this.#tmpTriangleNormals.length !== this.numberOfTriangles && (typeof this.#normalsValid === 'undefined' ||
      typeof this.#normalPerVertex === 'undefined'))
      return;
    if (this.numberOfTriangles !== this.#coordIndices.length / 3 || this.#coordIndices.length % 3 !== 0)
      return;
    if (this.#coordIndices.length !== this.#tmpTexIndices.length && this.#tmpTexIndices.length !== 0)
      return;

    const texCoordSize = typeof texCoord !== 'undefined' ? texCoord.length : 0;
    const normalSize = typeof normal !== 'undefined' ? normal.length : 0;
    const coordSize = coord.length;

    // populate the vertex array
    let vertex = coord[0];
    this.#max[0] = vertex.x;
    this.#max[1] = vertex.y;
    this.#max[2] = vertex.z;
    this.#min[0] = this.#max[0];
    this.#min[1] = this.#max[1];
    this.#min[2] = this.#max[2];
    for (let i = 0; i < coordSize; ++i) {
      vertex = coord[i];

      const x = vertex.x;
      if (this.#max[0] < x)
        this.#max[0] = x;
      else if (this.#min[0] > x)
        this.#min[0] = x;

      const y = vertex.y;
      if (this.#max[1] < y)
        this.#max[1] = y;
      else if (this.#min[1] > y)
        this.#min[1] = y;

      const z = vertex.z;
      if (this.#max[2] < z)
        this.#max[2] = z;
      else if (this.#min[2] > z)
        this.#min[2] = z;

      this.#coordinates.push(x);
      this.#coordinates.push(y);
      this.#coordinates.push(z);
    }

    for (let t = 0; t < this.numberOfTriangles; ++t) { // foreach triangle
      const k = 3 * t;
      for (let v = 0; v < 3; ++v) { // foreach vertex
        const index = k + v;
        const indexCoord = this.#coordIndices[index];

        // compute the normal per vertex (from normal per triangle)
        if (!this.#normalsValid || !this.#normalPerVertex) {
          let triangleNormal = new WbVector3();
          const faceNormal = this.#tmpTriangleNormals[t];
          const linkedTriangles = this.#tmpVertexToTriangle[indexCoord];
          const ltSize = linkedTriangles.length;

          // stores the normals of the linked triangles which are already used.
          const linkedTriangleNormals = [];
          let linkedTriangleNormalsIndex = 0;

          for (let i = 0; i < ltSize; ++i) {
            const linkedTriangleIndex = linkedTriangles[i];
            if (linkedTriangleIndex >= 0 && linkedTriangleIndex < this.numberOfTriangles) {
              const linkedTriangleNormal = this.#tmpTriangleNormals[linkedTriangleIndex];
              // perform the creaseAngle check
              if (faceNormal.angle(linkedTriangleNormal) < creaseAngle) {
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
          this.#normals.push(triangleNormal.x);
          this.#normals.push(triangleNormal.y);
          this.#normals.push(triangleNormal.z);
        } else { // normal already defined per vertex
          const indexNormal = this.#tmpNormalIndices[index];
          if (indexNormal >= 0 && indexNormal < normalSize) {
            const nor = normal[indexNormal];
            this.#normals.push(nor.x);
            this.#normals.push(nor.y);
            this.#normals.push(nor.z);
          }
        }

        if (this.areTextureCoordinatesValid) {
          const indexTex = this.#tmpTexIndices[index];
          if (indexTex >= 0 && indexTex < texCoordSize) {
            const tex = texCoord[indexTex];
            this.#textureCoordinates.push(tex.x);
            this.#textureCoordinates.push(1.0 - tex.y);
          } else {
            // foreach texture coordinate component
            this.#textureCoordinates.push(0.5);
            this.#textureCoordinates.push(0.5);
          }
        }
      }
    }

    if (!this.areTextureCoordinatesValid)
      this.#setDefaultTextureCoordinates(coord);
  }

  #setDefaultTextureCoordinates(coord) {
    const minBound = new WbVector3(this.#min[0], this.#min[1], this.#min[2]);
    const maxBound = new WbVector3(this.#max[0], this.#max[1], this.#max[2]);
    const size = maxBound.sub(minBound);

    // compute size and find longest and second-longest dimensions for default mapping
    let longestDimension = -1;
    let secondLongestDimension = -1;
    for (let i = 0; i < 3; ++i) {
      if (longestDimension < 0 || size.get(i) > size.get(longestDimension)) {
        secondLongestDimension = longestDimension;
        longestDimension = i;
      } else if (secondLongestDimension < 0 || size.get(i) > size.get(secondLongestDimension))
        secondLongestDimension = i;
    }

    if (longestDimension < 0 || secondLongestDimension < 0)
      return;

    let index = 0;
    const vertices = [];
    for (let t = 0; t < this.numberOfTriangles; ++t) { // foreach triangle
      vertices[0] = coord[this.#coordIndices[index]];
      vertices[1] = coord[this.#coordIndices[index + 1]];
      vertices[2] = coord[this.#coordIndices[index + 2]];

      // compute face center and normal
      const edge1 = vertices[1].sub(vertices[0]);
      const edge2 = vertices[2].sub(vertices[0]);
      const normal = edge1.cross(edge2);
      normal.normalize();
      const origin = vertices[0].add(vertices[1]).add(vertices[2]).div(3.0);

      // compute intersection with the bounding box
      const faceNormal = new WbRay(origin, normal);
      let tmin, tmax;
      const result = faceNormal.intersects(minBound, maxBound, tmin, tmax);
      const faceIndex = WbBox.findIntersectedFace(minBound, maxBound, origin.add(normal.mul(result[1])));

      for (let v = 0; v < 3; ++v) { // foreach vertex
        // compute default texture mapping
        this.#textureCoordinates.push((vertices[v].get(longestDimension) - this.#min[longestDimension]) /
          size.get(longestDimension));
        this.#textureCoordinates.push(1.0 - (vertices[v].get(secondLongestDimension) - this.#min[secondLongestDimension]) /
          size.get(longestDimension));

        // compute non-recursive mapping
        const uv = WbBox.computeTextureCoordinate(minBound, maxBound, vertices[v], true, faceIndex);
        this.#nonRecursiveTextureCoordinates.push(uv.x);
        this.#nonRecursiveTextureCoordinates.push(uv.y);
      }

      index += 3;
    }
  }

  #cutTriangleIfNeeded(coord, tesselatedPolygon, triangleIndex) {
    const results = [];

    // find the three vertices of this triangle from the tesselated polygon
    const firstVertexIndex = tesselatedPolygon[triangleIndex].x;
    const secondVertexIndex = tesselatedPolygon[triangleIndex + 1].x;
    const thirdVertexIndex = tesselatedPolygon[triangleIndex + 2].x;

    const firstNormalIndex = tesselatedPolygon[triangleIndex].y;
    const secondNormalIndex = tesselatedPolygon[triangleIndex + 1].y;
    const thirdNormalIndex = tesselatedPolygon[triangleIndex + 2].y;

    const firstTexCoordIndex = tesselatedPolygon[triangleIndex].z;
    const secondTexCoordIndex = tesselatedPolygon[triangleIndex + 1].z;
    const thirdTexCoordIndex = tesselatedPolygon[triangleIndex + 2].z;

    // prepare triangle edges for snipping checks
    const firstEdgeStart = coord[firstVertexIndex];
    const firstEdgeEnd = coord[secondVertexIndex];

    const secondEdgeStart = coord[secondVertexIndex];
    const secondEdgeEnd = coord[thirdVertexIndex];

    const thirdEdgeStart = coord[thirdVertexIndex];
    const thirdEdgeEnd = coord[firstVertexIndex];

    const checkedIndices = new Set();
    // for all vertices not in this triangle
    for (let i = 0; i < tesselatedPolygon.length; i++) {
      // skip vertices from this triangle
      if (tesselatedPolygon[i].x === firstVertexIndex || tesselatedPolygon[i].x === secondVertexIndex ||
          tesselatedPolygon[i].x === thirdVertexIndex)
        continue;
      // skip vertices we've already checked
      else if (checkedIndices.has(tesselatedPolygon[i].x))
        continue;
      else if (coord[tesselatedPolygon[i].x].isOnEdgeBetweenVertices(firstEdgeStart, firstEdgeEnd)) {
        // case 1: vertex is on the first edge of the triangle

        // first triangle
        results.push(new WbVector3(firstVertexIndex, firstNormalIndex, firstTexCoordIndex));
        results.push(new WbVector3(tesselatedPolygon[i].x, tesselatedPolygon[i].y, tesselatedPolygon[i].z));
        results.push(new WbVector3(thirdVertexIndex, thirdNormalIndex, thirdTexCoordIndex));
        // second triangle
        results.push(new WbVector3(tesselatedPolygon[i].x, tesselatedPolygon[i].y, tesselatedPolygon[i].z));
        results.push(new WbVector3(secondVertexIndex, secondNormalIndex, secondTexCoordIndex));
        results.push(new WbVector3(thirdVertexIndex, thirdNormalIndex, thirdTexCoordIndex));
      } else if (coord[tesselatedPolygon[i].x].isOnEdgeBetweenVertices(secondEdgeStart, secondEdgeEnd)) {
        // case 2: vertex is on the second edge of the triangle

        // first triangle
        results.push(new WbVector3(secondVertexIndex, secondNormalIndex, secondTexCoordIndex));
        results.push(new WbVector3(tesselatedPolygon[i].x, tesselatedPolygon[i].y, tesselatedPolygon[i].z));
        results.push(new WbVector3(firstVertexIndex, firstNormalIndex, firstTexCoordIndex));
        // second triangle
        results.push(new WbVector3(tesselatedPolygon[i].x, tesselatedPolygon[i].y, tesselatedPolygon[i].z));
        results.push(new WbVector3(thirdVertexIndex, thirdNormalIndex, thirdTexCoordIndex));
        results.push(new WbVector3(firstVertexIndex, firstNormalIndex, firstTexCoordIndex));
      } else if (coord[tesselatedPolygon[i].x].isOnEdgeBetweenVertices(thirdEdgeStart, thirdEdgeEnd)) {
        // case 3: vertex is on the third edge of the triangle

        // first triangle
        results.push(new WbVector3(thirdVertexIndex, thirdNormalIndex, thirdTexCoordIndex));
        results.push(new WbVector3(tesselatedPolygon[i].x, tesselatedPolygon[i].y, tesselatedPolygon[i].z));
        results.push(new WbVector3(secondVertexIndex, secondNormalIndex, secondTexCoordIndex));
        // second triangle
        results.push(new WbVector3(tesselatedPolygon[i].x, tesselatedPolygon[i].y, tesselatedPolygon[i].z));
        results.push(new WbVector3(firstVertexIndex, firstNormalIndex, firstTexCoordIndex));
        results.push(new WbVector3(secondVertexIndex, secondNormalIndex, secondTexCoordIndex));
      }

      // add this vertex to the list of those already checked
      checkedIndices.add(tesselatedPolygon[i].x);
    }

    //  default - no need to cut the triangle, return it as-was
    if (results.length === 0) {
      results.push(new WbVector3(firstVertexIndex, firstNormalIndex, firstTexCoordIndex));
      results.push(new WbVector3(secondVertexIndex, secondNormalIndex, secondTexCoordIndex));
      results.push(new WbVector3(thirdVertexIndex, thirdNormalIndex, thirdTexCoordIndex));
    }

    return results;
  }
}
