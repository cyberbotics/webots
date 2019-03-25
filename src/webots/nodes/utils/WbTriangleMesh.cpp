// Copyright 1996-2019 Cyberbotics Ltd.
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

#include "WbTriangleMesh.hpp"

#include "WbBox.hpp"
#include "WbLog.hpp"
#include "WbMFInt.hpp"
#include "WbMFVector2.hpp"
#include "WbMFVector3.hpp"
#include "WbRay.hpp"
#include "WbTesselator.hpp"

#include <cassert>
#include <limits>

WbTriangleMesh::WbTriangleMesh() {
  cleanup();
}

WbTriangleMesh::~WbTriangleMesh() {
  cleanup();
}

void WbTriangleMesh::cleanup() {
  mValid = false;
  mTextureCoordinatesValid = false;
  mNTriangles = 0;

  mWarnings.clear();

  mCoordIndices.clear();
  mCoordIndices.reserve(0);
  mVertices.clear();
  mVertices.reserve(0);
  mScaledVertices.clear();
  mScaledVertices.reserve(0);
  mTextureCoordinates.clear();
  mTextureCoordinates.reserve(0);
  mNonRecursiveTextureCoordinates.clear();
  mNonRecursiveTextureCoordinates.reserve(0);
  mNormals.clear();
  mNormals.reserve(0);

  cleanupTmpArrays();

  // default values of the bounding box dimensions
  static const double MIN = std::numeric_limits<double>::min();
  static const double MAX = std::numeric_limits<double>::max();
  mMax[X] = MIN;
  mMin[X] = MAX;
  mMax[Y] = MIN;
  mMin[Y] = MAX;
  mMax[Z] = MIN;
  mMin[Z] = MAX;
}

void WbTriangleMesh::cleanupTmpArrays() {
  mTmpTexIndices.clear();
  mTmpTexIndices.reserve(0);
  mTmpTriangleNormals.clear();
  mTmpTriangleNormals.reserve(0);
  mTmpVertexToTriangle.clear();
  mTmpVertexToTriangle.reserve(0);
}

QString WbTriangleMesh::init(const WbMFVector3 *coord, const WbMFInt *coordIndex, const WbMFVector2 *texCoord,
                             const WbMFInt *texCoordIndex, double creaseAngle, bool counterClockwise) {
  cleanup();

  // initial obvious check
  if (!coord || coord->size() == 0)
    return QString(QObject::tr("'coord' is empty."));
  if (!coordIndex || coordIndex->size() == 0)
    return QString(QObject::tr("'coordIndex' is empty."));

  const int nTrianglesEstimation = estimateNumberOfTriangles(coordIndex);  // overestimation of the number of triangles
  mNTriangles = 0;                                                         // keep the number of triangles

  // determine if the texture coordinate seems valid or not
  // this value will be used to determine the content of mTextureCoordinates
  const bool isTexCoordDefined = texCoord && texCoord->size() > 0;
  const bool isTexCoordIndexDefined = texCoordIndex && texCoordIndex->size() > 0;

  mTextureCoordinatesValid = isTexCoordDefined;
  if (isTexCoordDefined && isTexCoordIndexDefined && texCoordIndex->size() != coordIndex->size()) {
    mWarnings.append(QObject::tr(
      "Invalid texture mapping: size of 'coordIndex' and 'texCoordIndex' mismatch. The default texture mapping is applied."));
    mTextureCoordinatesValid = false;
  }

  // memory allocation of the tmp arrays (overestimated)
  const int estimateSize = 3 * nTrianglesEstimation;
  mCoordIndices.reserve(estimateSize);
  const int vertexSize = 3 * coord->size();
  if (mTextureCoordinatesValid)
    mTmpTexIndices.reserve(estimateSize);
  mTmpTriangleNormals.reserve(nTrianglesEstimation);
  mTmpVertexToTriangle.reserve(estimateSize);

  // memory allocation of the arrays (overestimated)
  mVertices.reserve(vertexSize);
  mTextureCoordinates.reserve(2 * estimateSize);
  mNonRecursiveTextureCoordinates.reserve(2 * estimateSize);
  mNormals.reserve(3 * estimateSize);

  // passes to create the final arrays
  if (isTexCoordDefined && !isTexCoordIndexDefined)
    // if texCoordIndex is empty use coordIndex
    indicesPass(coord, coordIndex, coordIndex);
  else
    indicesPass(coord, coordIndex, texCoordIndex);
  mNTriangles = mCoordIndices.size() / 3;
  if (!counterClockwise)
    reverseIndexOrder();
  const QString error = tmpNormalsPass(coord);
  if (!error.isEmpty())
    return error;
  finalPass(coord, texCoord, creaseAngle);

  // unallocate the useless data
  cleanupTmpArrays();
  mVertices.reserve(mVertices.size());
  mTextureCoordinates.reserve(mTextureCoordinates.size());
  mNonRecursiveTextureCoordinates.reserve(mNonRecursiveTextureCoordinates.size());
  mNormals.reserve(mNormals.size());

  // final obvious check
  if (mNTriangles <= 0) {
    cleanup();
    return QString(QObject::tr("The triangle mesh has no valid quad and no valid triangle."));
  }

  // validity switch
  mValid = true;

  return QString("");
}

// populate mCoordIndices and mTmpTexIndices with valid indices
void WbTriangleMesh::indicesPass(const WbMFVector3 *coord, const WbMFInt *coordIndex, const WbMFInt *texCoordIndex) {
  assert(!mTextureCoordinatesValid || texCoordIndex);

  // parse coordIndex
  QList<QPair<int, int>> currentFaceIndices;  // keep the coord and tex indices of the current face
  const int coordIndexSize = coordIndex->size();

  for (int i = 0; i < coordIndexSize; ++i) {
    // get the current index
    const int index = coordIndex->item(i);

    // special case: last index not equal to -1
    // -> add a current index to the current face
    //    in order to have consistent data
    if (index != -1 && i == coordIndexSize - 1)
      currentFaceIndices.append(QPair<int, int>(index, mTextureCoordinatesValid ? texCoordIndex->item(i) : 0));
    const int cfiSize = currentFaceIndices.size();
    // add the current face
    if (index == -1 || i == coordIndexSize - 1) {
      // check the validity of the current face
      // by checking if the range of the new face indices is valid
      bool currentFaceValidity = true;
      for (int j = 0; j < cfiSize; ++j) {
        const int cfi = currentFaceIndices[j].first;
        if (cfi < 0 || cfi >= coord->size())
          currentFaceValidity = false;
      }

      // add a face
      // -> tesselate everything in order to optimize dummy user input -> ex: [0 0 0 -1], [0 -1] or [0 1 2 0 -1])
      if (currentFaceValidity) {
        QList<QPair<int, int>> tesselatorOutput;
        QList<WbVector3> tesselatorVectorInput;

        for (int j = 0; j < cfiSize; ++j) {
          const int cfi = currentFaceIndices[j].first;
          tesselatorVectorInput.append(coord->item(cfi));
        }

        const QString warning(WbTesselator::tesselate(currentFaceIndices, tesselatorVectorInput, tesselatorOutput));
        if (!warning.isEmpty())
          mWarnings.append(warning);

        const int toSize = tesselatorOutput.size();
        assert(toSize % 3 == 0);

        // we assume that GLU will give us back n-2 triangles for any polygon
        // it tesselates, so we can take shortcuts for triangles and quads
        // simplest case: polygon is triangle
        if (toSize == 3) {
          mCoordIndices.append(tesselatorOutput[0].first);
          mCoordIndices.append(tesselatorOutput[1].first);
          mCoordIndices.append(tesselatorOutput[2].first);

          if (mTextureCoordinatesValid) {
            mTmpTexIndices.append(tesselatorOutput[0].second);
            mTmpTexIndices.append(tesselatorOutput[1].second);
            mTmpTexIndices.append(tesselatorOutput[2].second);
          }
        }
        // polygon is quad (two triangles)
        else if (toSize == 6) {
          mCoordIndices.append(tesselatorOutput[0].first);
          mCoordIndices.append(tesselatorOutput[1].first);
          mCoordIndices.append(tesselatorOutput[2].first);
          mCoordIndices.append(tesselatorOutput[3].first);
          mCoordIndices.append(tesselatorOutput[4].first);
          mCoordIndices.append(tesselatorOutput[5].first);

          if (mTextureCoordinatesValid) {
            mTmpTexIndices.append(tesselatorOutput[0].second);
            mTmpTexIndices.append(tesselatorOutput[1].second);
            mTmpTexIndices.append(tesselatorOutput[2].second);
            mTmpTexIndices.append(tesselatorOutput[3].second);
            mTmpTexIndices.append(tesselatorOutput[4].second);
            mTmpTexIndices.append(tesselatorOutput[5].second);
          }
        }
        // 5+ vertex polygon
        // sometimes GLU will perform bizzarre tesselations of some polygons
        // that generate triangles with zero area. We remove these triangles
        // as they cause numerical errors, but removing them can cause
        // flickering holes to appear in the polygon. We need to stitch the gap
        // left by the now-missing triangle back together to re-close the polygon
        else {
          for (int j = 0; j < toSize; j += 3) {
            const WbVector3 &a = coord->item(tesselatorOutput[j].first);
            const WbVector3 &b = coord->item(tesselatorOutput[j + 1].first);
            const WbVector3 &c = coord->item(tesselatorOutput[j + 2].first);

            // check for colinear edges, and discard this triangle if found
            const WbVector3 d = b - a;
            const WbVector3 e = c - b;
            // don't append if edges are co-linear
            if (d.cross(e).almostEquals(WbVector3(), 0.00001))
              continue;
            // don't append if two vertices are on the same spot
            if (a == b || a == c || b == c) {
              WbLog::error(QObject::tr(
                "Duplicate vertices detected while triangulating mesh. "
                "Try opening your model in 3D modeling software and removing duplicate vertices, then re-importing."));
              continue;
            }
            // see if this triangle has any overlapping vertices and snip triangle to improve tesselation and fill holes
            const QList<QPair<int, int>> snippedIndices = cutTriangleIfNeeded(coord, tesselatorOutput, j);
            assert(snippedIndices.size() % 3 == 0);
            for (int k = 0; k < snippedIndices.size(); k += 3) {
              mCoordIndices.append(snippedIndices[k].first);
              mCoordIndices.append(snippedIndices[k + 1].first);
              mCoordIndices.append(snippedIndices[k + 2].first);
              if (mTextureCoordinatesValid) {
                mTmpTexIndices.append(snippedIndices[k].second);
                mTmpTexIndices.append(snippedIndices[k + 1].second);
                mTmpTexIndices.append(snippedIndices[k + 2].second);
              }
            }
          }
        }
      }
      // warning: the current face is invalid
      else {
        QString IndicesString("[");
        for (int j = 0; j < cfiSize; ++j) {
          if (j != 0)
            IndicesString.append(", ");
          IndicesString.append(QString::number(currentFaceIndices[j].first));
        }
        IndicesString.append("]");
        mWarnings.append(QObject::tr("Out-of-range index in: %1.").arg(IndicesString));
        ;
      }

      currentFaceIndices.clear();
    }
    // add a coordIndex to the currentFace
    else
      currentFaceIndices.append(QPair<int, int>(index, mTextureCoordinatesValid ? texCoordIndex->item(i) : 0));
  }

  assert(mCoordIndices.size() == mTmpTexIndices.size() || mTmpTexIndices.size() == 0);
  assert(mCoordIndices.size() % 3 == 0);
}

QList<QPair<int, int>> WbTriangleMesh::cutTriangleIfNeeded(const WbMFVector3 *coord,
                                                           const QList<QPair<int, int>> &tesselatedPolygon,
                                                           const int triangleIndex) {
  QList<QPair<int, int>> results;

  // find the three vertices of this triangle from the tesselated polygon
  const int firstVertexIndex = tesselatedPolygon[triangleIndex].first;
  const int secondVertexIndex = tesselatedPolygon[triangleIndex + 1].first;
  const int thirdVertexIndex = tesselatedPolygon[triangleIndex + 2].first;

  const int firstTexCoordIndex = tesselatedPolygon[triangleIndex].second;
  const int secondTexCoordIndex = tesselatedPolygon[triangleIndex + 1].second;
  const int thirdTexCoordIndex = tesselatedPolygon[triangleIndex + 2].second;

  // prepare triangle edges for snipping checks
  const WbVector3 &firstEdgeStart = coord->item(firstVertexIndex);
  const WbVector3 &firstEdgeEnd = coord->item(secondVertexIndex);

  const WbVector3 &secondEdgeStart = coord->item(secondVertexIndex);
  const WbVector3 &secondEdgeEnd = coord->item(thirdVertexIndex);

  const WbVector3 &thirdEdgeStart = coord->item(thirdVertexIndex);
  const WbVector3 &thirdEdgeEnd = coord->item(firstVertexIndex);

  QHash<int, bool> checkedIndices;
  // for all vertices not in this triangle
  for (int i = 0; i < tesselatedPolygon.size(); ++i) {
    // skip vertices from this triangle
    if (tesselatedPolygon[i].first == firstVertexIndex || tesselatedPolygon[i].first == secondVertexIndex ||
        tesselatedPolygon[i].first == thirdVertexIndex)
      continue;
    // skip vertices we've already checked
    else if (checkedIndices.value(tesselatedPolygon[i].first))
      continue;

    // case 1: vertex is on the first edge of the triangle
    else if (coord->item(tesselatedPolygon[i].first).isOnEdgeBetweenVertices(firstEdgeStart, firstEdgeEnd)) {
      // first triangle
      results.append(QPair<int, int>(firstVertexIndex, firstTexCoordIndex));
      results.append(QPair<int, int>(tesselatedPolygon[i].first, tesselatedPolygon[i].second));
      results.append(QPair<int, int>(thirdVertexIndex, thirdTexCoordIndex));
      // second triangle
      results.append(QPair<int, int>(tesselatedPolygon[i].first, tesselatedPolygon[i].second));
      results.append(QPair<int, int>(secondVertexIndex, secondTexCoordIndex));
      results.append(QPair<int, int>(thirdVertexIndex, thirdTexCoordIndex));
    }
    // case 2: vertex is on the second edge of the triangle
    else if (coord->item(tesselatedPolygon[i].first).isOnEdgeBetweenVertices(secondEdgeStart, secondEdgeEnd)) {
      // first triangle
      results.append(QPair<int, int>(secondVertexIndex, secondTexCoordIndex));
      results.append(QPair<int, int>(tesselatedPolygon[i].first, tesselatedPolygon[i].second));
      results.append(QPair<int, int>(firstVertexIndex, firstTexCoordIndex));
      // second triangle
      results.append(QPair<int, int>(tesselatedPolygon[i].first, tesselatedPolygon[i].second));
      results.append(QPair<int, int>(thirdVertexIndex, thirdTexCoordIndex));
      results.append(QPair<int, int>(firstVertexIndex, firstTexCoordIndex));
    }
    // case 3: vertex is on the third edge of the triangle
    else if (coord->item(tesselatedPolygon[i].first).isOnEdgeBetweenVertices(thirdEdgeStart, thirdEdgeEnd)) {
      // first triangle
      results.append(QPair<int, int>(thirdVertexIndex, thirdTexCoordIndex));
      results.append(QPair<int, int>(tesselatedPolygon[i].first, tesselatedPolygon[i].second));
      results.append(QPair<int, int>(secondVertexIndex, secondTexCoordIndex));
      // second triangle
      results.append(QPair<int, int>(tesselatedPolygon[i].first, tesselatedPolygon[i].second));
      results.append(QPair<int, int>(firstVertexIndex, firstTexCoordIndex));
      results.append(QPair<int, int>(secondVertexIndex, secondTexCoordIndex));
    }

    // add this vertex to the list of those already checked
    checkedIndices.insert(tesselatedPolygon[i].first, true);
  }
  //  default - no need to cut the triangle, return it as-was
  if (results.isEmpty()) {
    results.append(QPair<int, int>(firstVertexIndex, firstTexCoordIndex));
    results.append(QPair<int, int>(secondVertexIndex, secondTexCoordIndex));
    results.append(QPair<int, int>(thirdVertexIndex, thirdTexCoordIndex));
  }

  return results;
}

// populate mTmpTriangleNormals from coord and mCoordIndices
QString WbTriangleMesh::tmpNormalsPass(const WbMFVector3 *coord) {
  assert(mNTriangles == mCoordIndices.size() / 3);
  assert(mCoordIndices.size() % 3 == 0);

  // 1. compute normals per triangle
  for (int i = 0; i < mNTriangles; ++i) {
    const int j = 3 * i;
    const int indexA = mCoordIndices[j];
    const int indexB = mCoordIndices[j + 1];
    const int indexC = mCoordIndices[j + 2];

    assert(indexA >= 0 && indexA < coord->size());
    assert(indexB >= 0 && indexB < coord->size());
    assert(indexC >= 0 && indexC < coord->size());

    const WbVector3 &posA = coord->item(indexA);
    const WbVector3 &posB = coord->item(indexB);
    const WbVector3 &posC = coord->item(indexC);

    const WbVector3 &v1 = posB - posA;
    const WbVector3 &v2 = posC - posA;
    WbVector3 normal(v1.cross(v2));
    const double length = normal.length();
    if (length == 0.0)
      return QObject::tr("Null normal for face %1 %2 %3.\n This can be caused by duplicate vertices in your mesh. "
                         "Try opening your model in 3D modeling software and removing duplicate vertices, then re-importing.")
        .arg(indexA)
        .arg(indexB)
        .arg(indexC);
    normal /= length;
    mTmpTriangleNormals.append(normal);
  }

  assert(mTmpTriangleNormals.size() == mNTriangles);

  // 2. compute the map coordIndex->triangleIndex
  for (int t = 0; t < mNTriangles; ++t) {
    const int k = 3 * t;
    int index = mCoordIndices[k];
    mTmpVertexToTriangle.insert(index, t);
    index = mCoordIndices[k + 1];
    mTmpVertexToTriangle.insert(index, t);
    index = mCoordIndices[k + 2];
    mTmpVertexToTriangle.insert(index, t);
  }
  return "";
}

// populate mTextureCoordinates and mNonRecursiveTextureCoordinates with default values
void WbTriangleMesh::setDefaultTextureCoordinates(const WbMFVector3 *coord) {
  const WbVector3 minBound(min(X), min(Y), min(Z));
  const WbVector3 maxBound(max(X), max(Y), max(Z));
  const WbVector3 size(maxBound - minBound);

  // compute size and find longest and second-longest dimensions for default mapping
  int longestDimension = -1;
  int secondLongestDimension = -1;
  for (int i = 0; i < 3; ++i) {
    if (longestDimension < 0 || size[i] > size[longestDimension]) {
      secondLongestDimension = longestDimension;
      longestDimension = i;
    } else if (secondLongestDimension < 0 || size[i] > size[secondLongestDimension])
      secondLongestDimension = i;
  }

  assert(longestDimension >= 0 && secondLongestDimension >= 0);

  int index = 0;
  WbVector3 vertices[3];
  for (int t = 0; t < mNTriangles; ++t) {  // foreach triangle
    vertices[0] = coord->item(mCoordIndices[index]);
    vertices[1] = coord->item(mCoordIndices[index + 1]);
    vertices[2] = coord->item(mCoordIndices[index + 2]);

    // compute face center and normal
    const WbVector3 edge1(vertices[1] - vertices[0]);
    const WbVector3 edge2(vertices[2] - vertices[0]);
    WbVector3 normal(edge1.cross(edge2));
    normal.normalize();
    const WbVector3 origin((vertices[0] + vertices[1] + vertices[2]) / 3.0);

    // compute intersection with the bounding box
    const WbRay faceNormal(origin, normal);
    double tmin, tmax;
    const std::pair<bool, double> result = faceNormal.intersects(minBound, maxBound, tmin, tmax);
    assert(result.first);
    const int faceIndex = WbBox::findIntersectedFace(minBound, maxBound, origin + result.second * normal);

    for (int v = 0; v < 3; ++v) {  // foreach vertex
      // compute default texture mapping
      mTextureCoordinates.append((vertices[v][longestDimension] - min(longestDimension)) / size[longestDimension]);
      mTextureCoordinates.append(1.0 -
                                 (vertices[v][secondLongestDimension] - min(secondLongestDimension)) / size[longestDimension]);

      // compute non-recursive mapping
      const WbVector2 uv(WbBox::computeTextureCoordinate(minBound, maxBound, vertices[v], true, faceIndex));
      mNonRecursiveTextureCoordinates.append(uv.x());
      mNonRecursiveTextureCoordinates.append(uv.y());
    }

    index += 3;
  }
}

// populate mIndices, mVertices, mTextureCoordinates and mNormals
void WbTriangleMesh::finalPass(const WbMFVector3 *coord, const WbMFVector2 *texCoord, double creaseAngle) {
  assert(coord && coord->size() > 0);
  assert(mTmpTriangleNormals.size() == mNTriangles);
  assert(mNTriangles == mCoordIndices.size() / 3);
  assert(mCoordIndices.size() % 3 == 0);
  assert(mCoordIndices.size() == mTmpTexIndices.size() || mTmpTexIndices.size() == 0);
  const int texCoordSize = texCoord ? texCoord->size() : 0;
  const int coordSize = coord->size();

  // populate the vertex array
  WbVector3 v = coord->item(0);
  mMax[X] = v.x();
  mMax[Y] = v.y();
  mMax[Z] = v.z();
  mMin[X] = mMax[X];
  mMin[Y] = mMax[Y];
  mMin[Z] = mMax[Z];
  for (int i = 0; i < coordSize; ++i) {
    v = coord->item(i);

    const double x = v.x();
    if (mMax[X] < x)
      mMax[X] = x;
    else if (mMin[X] > x)
      mMin[X] = x;

    const double y = v.y();
    if (mMax[Y] < y)
      mMax[Y] = y;
    else if (mMin[Y] > y)
      mMin[Y] = y;

    const double z = v.z();
    if (mMax[Z] < z)
      mMax[Z] = z;
    else if (mMin[Z] > z)
      mMin[Z] = z;

    mVertices.append(x);
    mVertices.append(y);
    mVertices.append(z);
  }

  for (int t = 0; t < mNTriangles; ++t) {  // foreach triangle
    const int k = 3 * t;
    for (int v = 0; v < 3; ++v) {  // foreach vertex
      const int index = k + v;
      const int indexCoord = mCoordIndices[index];

      // compute the normal
      WbVector3 normal;
      const WbVector3 &faceNormal = mTmpTriangleNormals[t];
      const QList<int> &linkedTriangles = mTmpVertexToTriangle.values(indexCoord);
      const int ltSize = linkedTriangles.size();
      // stores the normals of the linked triangles which are already used.
      const WbVector3 **linkedTriangleNormals = new const WbVector3 *[ltSize];
      int linkedTriangleNormalsIndex = 0;
      for (int i = 0; i < ltSize; ++i) {
        const int linkedTriangleIndex = linkedTriangles.at(i);
        if (linkedTriangleIndex >= 0 && linkedTriangleIndex < mNTriangles) {
          const WbVector3 &linkedTriangleNormal = mTmpTriangleNormals[linkedTriangleIndex];
          // perform the creaseAngle check
          if (faceNormal.angle(linkedTriangleNormal) < creaseAngle) {
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
              normal += linkedTriangleNormal;
              linkedTriangleNormals[linkedTriangleNormalsIndex] = &linkedTriangleNormal;
              linkedTriangleNormalsIndex++;
            }
          }
        }
      }
      delete[] linkedTriangleNormals;

      if (normal.isNull())
        normal = faceNormal;
      else
        normal.normalize();

      // populate the remaining two final arrays

      mNormals.append(normal[X]);
      mNormals.append(normal[Y]);
      mNormals.append(normal[Z]);

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
  assert(mVertices.size() == 3 * coordSize);
  assert(mNormals.size() == 3 * 3 * mNTriangles);
  assert(mTextureCoordinates.size() == 0 || mTextureCoordinates.size() == 2 * 3 * mNTriangles);
  assert(mNonRecursiveTextureCoordinates.size() == 0 || mNonRecursiveTextureCoordinates.size() == 2 * 3 * mNTriangles);
}

// reverse the order of the second and third element
// of each triplet of the mCoordIndices and mTmpTexIndices arrays
void WbTriangleMesh::reverseIndexOrder() {
  const int coordIndicesSize = mCoordIndices.size();
  assert(coordIndicesSize % 3 == 0);
  assert(coordIndicesSize == mTmpTexIndices.size() || mTmpTexIndices.size() == 0);

  for (int i = 0; i < coordIndicesSize; i += 3) {
    const int i1 = i + 1;
    const int i2 = i + 2;
    const int third = mCoordIndices.at(i2);
    mCoordIndices[i2] = mCoordIndices.at(i1);
    mCoordIndices[i1] = third;

    if (mTextureCoordinatesValid) {
      const int third = mTmpTexIndices.at(i2);
      mTmpTexIndices[i2] = mTmpTexIndices.at(i1);
      mTmpTexIndices[i1] = third;
    }
  }
}

// Return an overestimation of the number of triangles by parsing
// the coordIndex according to this rule:
//
// coordIndex [
//   -1             # nop
//   0 -1           # nop
//   0 1 -1         # nop
//   0 1 2 -1       # +1 triangle
//   0 0 0 -1       # +1 triangle (instead of nop -> overestimation of a dummy user input)
//   0 1 2 3 -1     # +2 triangles
//   0 1 2 3 0 -1   # +3 triangles (instead of +2 -> overestimation of a dummy user input)
//   0 1 2 [-1]     # +1 triangle (don't take into account the latest -1)
// ]
int WbTriangleMesh::estimateNumberOfTriangles(const WbMFInt *coordIndex) {
  assert(coordIndex);

  WbMFInt::Iterator coordIndexIt(coordIndex);
  int nTriangles = 0;
  int currentFaceIndicesCounter = 0;
  while (coordIndexIt.hasNext()) {
    const int index = coordIndexIt.next();
    if (index != -1 && !coordIndexIt.hasNext())
      ++currentFaceIndicesCounter;

    if (index == -1 || !coordIndexIt.hasNext()) {
      int nCurrentFaceTriangle = qMax(0, currentFaceIndicesCounter - 2);
      nTriangles += nCurrentFaceTriangle;
      currentFaceIndicesCounter = 0;
    } else
      ++currentFaceIndicesCounter;
  }
  return nTriangles;
}

void WbTriangleMesh::updateScaledVertices(double x, double y, double z) {
  const int n = mVertices.size();
  assert(n % 3 == 0);
  mScaledVertices.resize(n);
  int i = 0;
  while (i < n) {
    mScaledVertices[i] = x * mVertices.at(i);
    ++i;
    mScaledVertices[i] = y * mVertices.at(i);
    ++i;
    mScaledVertices[i] = z * mVertices.at(i);
    ++i;
  }
}
