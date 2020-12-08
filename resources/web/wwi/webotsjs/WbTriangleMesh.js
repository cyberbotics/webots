class WbTriangleMesh {
  constructor(){
    this.isValid = false;
    this.areTextureCoordinatesValid;
    this.normalsValid;

    this.numberOfTriangles;
    this.normalPerVertex;

    this.coordinates;
    this.coordIndices;
    this.tmpTexIndices;

    this.textureCoordinates;
    this.nonRecursiveTextureCoordinates;
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

  textureCoordinate(triangle, vertex, component) {
    return this.textureCoordinates[2 * index(triangle, vertex) + component];
  }

  nonRecursiveTextureCoordinate(triangle, vertex, component) {
    return this.nonRecursiveTextureCoordinates[2 * index(triangle, vertex) + component];
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

  // populate mCoordIndices and mTmpTexIndices with valid indices
  indicesPass(coord, coordIndex, normalIndex, texCoordIndex) {
    assert(!this.normalsValid || typeof normalIndex === 'undefined');
    assert(!this.textureCoordinatesValid || typeof texCoordIndex === 'undefined');
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
        currentFaceIndices.push(new WbVector3(index, this.NormalsValid ? normalIndex[i] : 0, this.textureCoordinatesValid ? texCoordIndex[i] : 0));
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

            if (this.textureCoordinatesValid) {
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

            if (this.textureCoordinatesValid) {
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
                if (this.textureCoordinatesValid) {
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
        currentFaceIndices.push(new WbVector3(index, this.normalsValid ? normalIndex[i] : 0, this.textureCoordinatesValid ? texCoordIndex[i] : 0));
    }

    assert(this.coordIndices.length === this.tmpNormalIndices.length || this.tmpNormalIndices.length === 0);
    assert(this.coordIndices.length === this.tmpTexIndices.length || this.tmpTexIndices.length === 0);
    assert(this.coordIndices.length % 3 === 0);
  }

  // reverse the order of the second and third element
  // of each triplet of the mCoordIndices and mTmpTexIndices arrays
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

      if (this.textureCoordinatesValid) {
        const thirdIndex = this.tmpTexIndices[i2];
        this.tmpTexIndices[i2] = this.tmpTexIndices[i1];
        this.tmpTexIndices[i1] = thirdIndex;
      }
    }
  }

  // populate mTmpTriangleNormals from coord and mCoordIndices
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

        assert(indexA >= 0 && indexA < coord.length;
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
        this.tmpTriangleNormals.append(n);
      }
    }

    assert(this.tmpTriangleNormals.length === this.numberOfTriangles);

    // 2. compute the map coordIndex->triangleIndex
    for (let t = 0; t < this.numberOfTriangles; ++t) {
      const k = 3 * t;
      let index = this.coordIndices[k];
      this.tmpVertexToTriangle[index] = t;
      index = this.coordIndices[k + 1];
      this.tmpVertexToTriangle[index] = t;
      index = this.coordIndices[k + 2];
      this.tmpVertexToTriangle[index] = t;
    }

    return "";
  }

/*
  // populate mIndices, mCoordinates, mTextureCoordinates and mNormals
  finalPass(coord, normal, texCoord, creaseAngle) {
    assert(coord && coord.length > 0);
    assert(this.tmpTriangleNormals.length === mNTriangles || (this.normalsValid && this.normalPerVertex));
    assert(this.numberOfTriangles === mCoordIndices.length / 3);
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
      vertex = coord->item(i);

      const double x = vertex.x();
      if (mMax[X] < x)
        mMax[X] = x;
      else if (mMin[X] > x)
        mMin[X] = x;

      const double y = vertex.y();
      if (mMax[Y] < y)
        mMax[Y] = y;
      else if (mMin[Y] > y)
        mMin[Y] = y;

      const double z = vertex.z();
      if (mMax[Z] < z)
        mMax[Z] = z;
      else if (mMin[Z] > z)
        mMin[Z] = z;

      mCoordinates.append(x);
      mCoordinates.append(y);
      mCoordinates.append(z);
    }

    for (int t = 0; t < mNTriangles; ++t) {  // foreach triangle
      const int k = 3 * t;
      for (int v = 0; v < 3; ++v) {  // foreach vertex
        const int index = k + v;
        const int indexCoord = mCoordIndices[index];

        // compute the normal per vertex (from normal per triangle)
        if (!mNormalsValid || !mNormalPerVertex) {
          WbVector3 triangleNormal;
          const WbVector3 &faceNormal = mTmpTriangleNormals[t];
          const QList<int> &linkedTriangles = mTmpVertexToTriangle.values(indexCoord);
          const int ltSize = linkedTriangles.size();
          // stores the normals of the linked triangles which are already used.
          const WbVector3 **linkedTriangleNormals = new const WbVector3 *[ltSize];
          int creasedLinkedTriangleNumber = 0;
          int linkedTriangleNormalsIndex = 0;
          for (int i = 0; i < ltSize; ++i) {
            const int linkedTriangleIndex = linkedTriangles.at(i);
            if (linkedTriangleIndex >= 0 && linkedTriangleIndex < mNTriangles) {
              const WbVector3 &linkedTriangleNormal = mTmpTriangleNormals[linkedTriangleIndex];
              // perform the creaseAngle check
              if (faceNormal.angle(linkedTriangleNormal) < creaseAngle) {
                creasedLinkedTriangleNumber++;
                bool found = false;
                // we don't want coplanar face normals on e.g. a cylinder to bias a
                // normal and cause discontinuities, so don't include duplicated
                // normals in the smoothing pass
                for (int lN = 0; lN < linkedTriangleNormalsIndex; ++lN) {
                  const WbVector3 *currentLinkedTriangleNormal = linkedTriangleNormals[lN];
                  if (currentLinkedTriangleNormal->almostEquals(linkedTriangleNormal, 0.0001)) {
                    found = true;
                    break;
                  }
                }
                if (!found) {
                  triangleNormal += linkedTriangleNormal;
                  linkedTriangleNormals[linkedTriangleNormalsIndex] = &linkedTriangleNormal;
                  linkedTriangleNormalsIndex++;
                }
              }
            }
          }
          delete[] linkedTriangleNormals;

          if (triangleNormal.isNull())
            triangleNormal = faceNormal;
          else
            triangleNormal.normalize();

          // populate the remaining two final arrays
          mNormals.append(triangleNormal[X]);
          mNormals.append(triangleNormal[Y]);
          mNormals.append(triangleNormal[Z]);
          mIsNormalCreased.append(creasedLinkedTriangleNumber == ltSize);
        } else {  // normal already defined per vertex
          const int indexNormal = mTmpNormalIndices[index];
          if (indexNormal >= 0 && indexNormal < normalSize) {
            const WbVector3 nor(normal->item(indexNormal));
            mNormals.append(nor.x());
            mNormals.append(nor.y());
            mNormals.append(nor.z());
            mIsNormalCreased.append(false);
          }
        }

        if (mTextureCoordinatesValid) {
          const int indexTex = mTmpTexIndices[index];
          if (indexTex >= 0 && indexTex < texCoordSize) {
            const WbVector2 tex(texCoord->item(indexTex));
            mTextureCoordinates.append(tex.x());
            mTextureCoordinates.append(1.0 - tex.y());
          } else {
            // foreach texture coordinate component
            mTextureCoordinates.append(0.5);
            mTextureCoordinates.append(0.5);
          }
        }
      }
    }

    if (!mTextureCoordinatesValid)
      setDefaultTextureCoordinates(coord);

    // check the resulted size
    assert(mCoordinates.size() == 3 * coordSize);
    assert(mNormals.size() == 3 * 3 * mNTriangles);
    assert(mIsNormalCreased.size() == 3 * mNTriangles);
    assert(mTextureCoordinates.size() == 0 || mTextureCoordinates.size() == 2 * 3 * mNTriangles);
    assert(mNonRecursiveTextureCoordinates.size() == 0 || mNonRecursiveTextureCoordinates.size() == 2 * 3 * mNTriangles);
  }
*/
}

export {WbTriangleMesh}
