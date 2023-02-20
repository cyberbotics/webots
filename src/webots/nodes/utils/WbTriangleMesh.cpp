// Copyright 1996-2023 Cyberbotics Ltd.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     https://www.apache.org/licenses/LICENSE-2.0
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
  mNormalsValid = false;
  mNormalPerVertex = true;
  mTextureCoordinatesValid = false;
  mNTriangles = 0;

  mWarnings.clear();

  mCoordIndices.clear();
  mCoordIndices.reserve(0);
  mCoordinates.clear();
  mCoordinates.reserve(0);
  mScaledCoordinates.clear();
  mScaledCoordinates.reserve(0);
  mTextureCoordinates.clear();
  mTextureCoordinates.reserve(0);
  mNonRecursiveTextureCoordinates.clear();
  mNonRecursiveTextureCoordinates.reserve(0);
  mNormals.clear();
  mNormals.reserve(0);
  mIsNormalCreased.clear();
  mIsNormalCreased.reserve(0);

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
  mTmpNormalIndices.clear();
  mTmpNormalIndices.reserve(0);
  mTmpTriangleNormals.clear();
  mTmpTriangleNormals.reserve(0);
  mTmpVertexToTriangle.clear();
  mTmpVertexToTriangle.reserve(0);
}

QString WbTriangleMesh::init(const WbMFVector3 *coord, const WbMFInt *coordIndex, const WbMFVector3 *normal,
                             const WbMFInt *normalIndex, const WbMFVector2 *texCoord, const WbMFInt *texCoordIndex,
                             double creaseAngle, bool normalPerVertex) {
  cleanup();

  mNormalPerVertex = normalPerVertex;

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
    mWarnings.append(QObject::tr("Invalid texture mapping: the sizes of 'coordIndex' and 'texCoordIndex' mismatch. The default "
                                 "texture mapping is applied."));
    mTextureCoordinatesValid = false;
  }

  // determine if the normal seems valid or not
  // this value will be used to determine the content of mNormals
  const bool isNormalDefined = normal && normal->size() > 0;
  const bool isNormalIndexDefined = normalIndex && normalIndex->size() > 0;

  mNormalsValid = isNormalDefined;
  if (mNormalPerVertex) {
    if (isNormalDefined && isNormalIndexDefined && normalIndex->size() != coordIndex->size()) {
      mWarnings.append(
        QObject::tr("Invalid normal definition: the sizes of 'coordIndex' and 'normalIndex' mismatch. The normals will "
                    "be computed using the creaseAngle."));
      mNormalsValid = false;
    }
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
  mCoordinates.reserve(vertexSize);
  mTextureCoordinates.reserve(2 * estimateSize);
  mNonRecursiveTextureCoordinates.reserve(2 * estimateSize);
  mNormals.reserve(3 * estimateSize);
  mIsNormalCreased.reserve(estimateSize);

  // passes to create the final arrays
  indicesPass(coord, coordIndex, (mNormalsValid && mNormalPerVertex && isNormalIndexDefined) ? normalIndex : coordIndex,
              (isTexCoordDefined && isTexCoordIndexDefined) ? texCoordIndex : coordIndex);
  mNTriangles = mCoordIndices.size() / 3;
  if (mNormalsValid && !mNormalPerVertex && mNTriangles > normal->size()) {
    mWarnings.append(QObject::tr("Invalid normal definition: the size of 'normal' should equal the number of triangles when "
                                 "'normalPerVertex' is FALSE. The normals will be computed using the creaseAngle."));
    mNormalsValid = false;
  }
  const QString error = tmpNormalsPass(coord, normal);
  if (!error.isEmpty())
    return error;
  finalPass(coord, normal, texCoord, creaseAngle);

  // unallocate the useless data
  cleanupTmpArrays();
  mCoordinates.reserve(mCoordinates.size());
  mTextureCoordinates.reserve(mTextureCoordinates.size());
  mNonRecursiveTextureCoordinates.reserve(mNonRecursiveTextureCoordinates.size());
  mNormals.reserve(mNormals.size());
  mIsNormalCreased.reserve(mIsNormalCreased.size());

  // final obvious check
  if (mNTriangles <= 0) {
    cleanup();
    return QString(QObject::tr("The triangle mesh has no valid quad and no valid triangle."));
  }

  // validity switch
  mValid = true;

  return QString("");
}

QString WbTriangleMesh::init(const double *coord, const double *normal, const double *texCoord, const unsigned int *index,
                             int numberOfVertices, int indexSize) {
  cleanup();
  // validity switch
  mValid = true;
  if (numberOfVertices == 0)
    return QString();

  mTextureCoordinatesValid = texCoord != NULL;
  mNormalsValid = normal != NULL;
  mNTriangles = indexSize / 3;

  mCoordIndices.reserve(indexSize);
  mCoordinates.reserve(3 * numberOfVertices);
  mScaledCoordinates.reserve(3 * numberOfVertices);
  if (mTextureCoordinatesValid) {
    mTextureCoordinates.reserve(2 * numberOfVertices);
    mNonRecursiveTextureCoordinates.reserve(2 * numberOfVertices);
  }
  mNormals.reserve(3 * numberOfVertices);
  mIsNormalCreased.reserve(numberOfVertices);

  for (int i = 0; i < indexSize; ++i)
    mCoordIndices.append(index[i]);

  for (int i = 0; i < numberOfVertices; ++i) {
    const double x = coord[3 * i];
    if (mMax[X] < x)
      mMax[X] = x;
    else if (mMin[X] > x)
      mMin[X] = x;

    const double y = coord[3 * i + 1];
    if (mMax[Y] < y)
      mMax[Y] = y;
    else if (mMin[Y] > y)
      mMin[Y] = y;

    const double z = coord[3 * i + 2];
    if (mMax[Z] < z)
      mMax[Z] = z;
    else if (mMin[Z] > z)
      mMin[Z] = z;

    mCoordinates.append(x);
    mCoordinates.append(y);
    mCoordinates.append(z);
    mScaledCoordinates.append(x);
    mScaledCoordinates.append(y);
    mScaledCoordinates.append(z);
  }

  if (mTextureCoordinatesValid) {
    for (int t = 0; t < mNTriangles; ++t) {  // foreach triangle
      for (int v = 0; v < 3; ++v) {          // foreach vertex
        const int currentIndex = mCoordIndices[3 * t + v];
        mTextureCoordinates.append(texCoord[2 * currentIndex]);
        mTextureCoordinates.append(texCoord[2 * currentIndex + 1]);
        mNonRecursiveTextureCoordinates.append(texCoord[2 * currentIndex]);
        mNonRecursiveTextureCoordinates.append(texCoord[2 * currentIndex + 1]);
        mNormals.append(normal[3 * currentIndex]);
        mNormals.append(normal[3 * currentIndex + 1]);
        mNormals.append(normal[3 * currentIndex + 2]);
        mIsNormalCreased.append(false);
      }
    }
  }

  return QString("");
}

// populate mCoordIndices and mTmpTexIndices with valid indices
void WbTriangleMesh::indicesPass(const WbMFVector3 *coord, const WbMFInt *coordIndex, const WbMFInt *normalIndex,
                                 const WbMFInt *texCoordIndex) {
  assert(!mNormalsValid || normalIndex);
  assert(!mTextureCoordinatesValid || texCoordIndex);
  assert(mTmpNormalIndices.size() == 0);
  assert(mTmpTexIndices.size() == 0);

  // parse coordIndex
  QList<QVector<int>> currentFaceIndices;  // keep the coord, normal and tex indices of the current face
  const int coordIndexSize = coordIndex->size();

  for (int i = 0; i < coordIndexSize; ++i) {
    // get the current index
    const int id = coordIndex->item(i);

    // special case: last index not equal to -1
    // -> add a current index to the current face
    //    in order to have consistent data
    if (id != -1 && i == coordIndexSize - 1)
      currentFaceIndices.append(QVector<int>() << id << (mNormalsValid ? normalIndex->item(i) : 0)
                                               << (mTextureCoordinatesValid ? texCoordIndex->item(i) : 0));
    const int cfiSize = currentFaceIndices.size();
    // add the current face
    if (id == -1 || i == coordIndexSize - 1) {
      // check the validity of the current face
      // by checking if the range of the new face indices is valid
      bool currentFaceValidity = true;
      for (int j = 0; j < cfiSize; ++j) {
        const int cfi = currentFaceIndices[j][0];
        if (cfi < 0 || cfi >= coord->size())
          currentFaceValidity = false;
      }

      // add a face
      // -> tesselate everything in order to optimize dummy user input -> ex: [0 0 0 -1], [0 -1] or [0 1 2 0 -1])
      if (currentFaceValidity) {
        QList<QVector<int>> tesselatorOutput;
        QList<WbVector3> tesselatorVectorInput;

        for (int j = 0; j < cfiSize; ++j) {
          const int cfi = currentFaceIndices[j][0];
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
          mCoordIndices.append(tesselatorOutput[0][0]);
          mCoordIndices.append(tesselatorOutput[1][0]);
          mCoordIndices.append(tesselatorOutput[2][0]);

          if (mNormalsValid) {
            mTmpNormalIndices.append(tesselatorOutput[0][1]);
            mTmpNormalIndices.append(tesselatorOutput[1][1]);
            mTmpNormalIndices.append(tesselatorOutput[2][1]);
          }

          if (mTextureCoordinatesValid) {
            mTmpTexIndices.append(tesselatorOutput[0][2]);
            mTmpTexIndices.append(tesselatorOutput[1][2]);
            mTmpTexIndices.append(tesselatorOutput[2][2]);
          }
        }
        // polygon is quad (two triangles)
        else if (toSize == 6) {
          mCoordIndices.append(tesselatorOutput[0][0]);
          mCoordIndices.append(tesselatorOutput[1][0]);
          mCoordIndices.append(tesselatorOutput[2][0]);
          mCoordIndices.append(tesselatorOutput[3][0]);
          mCoordIndices.append(tesselatorOutput[4][0]);
          mCoordIndices.append(tesselatorOutput[5][0]);

          if (mNormalsValid) {
            mTmpNormalIndices.append(tesselatorOutput[0][1]);
            mTmpNormalIndices.append(tesselatorOutput[1][1]);
            mTmpNormalIndices.append(tesselatorOutput[2][1]);
            mTmpNormalIndices.append(tesselatorOutput[3][1]);
            mTmpNormalIndices.append(tesselatorOutput[4][1]);
            mTmpNormalIndices.append(tesselatorOutput[5][1]);
          }

          if (mTextureCoordinatesValid) {
            mTmpTexIndices.append(tesselatorOutput[0][2]);
            mTmpTexIndices.append(tesselatorOutput[1][2]);
            mTmpTexIndices.append(tesselatorOutput[2][2]);
            mTmpTexIndices.append(tesselatorOutput[3][2]);
            mTmpTexIndices.append(tesselatorOutput[4][2]);
            mTmpTexIndices.append(tesselatorOutput[5][2]);
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
            const WbVector3 &a = coord->item(tesselatorOutput[j][0]);
            const WbVector3 &b = coord->item(tesselatorOutput[j + 1][0]);
            const WbVector3 &c = coord->item(tesselatorOutput[j + 2][0]);

            // check for colinear edges, and discard this triangle if found
            const WbVector3 d = b - a;
            const WbVector3 e = c - b;
            // don't append if edges are co-linear
            if (d.cross(e).almostEquals(WbVector3(), 0.00001))
              continue;
            // don't append if two vertices are on the same spot
            if (a == b || a == c || b == c) {
              WbLog::error(
                QObject::tr(
                  "Duplicate vertices detected while triangulating mesh. "
                  "Try opening your model in 3D modeling software and removing duplicate vertices, then re-importing."),
                false, WbLog::PARSING);
              continue;
            }
            // see if this triangle has any overlapping vertices and snip triangle to improve tesselation and fill holes
            const QList<QVector<int>> snippedIndices = cutTriangleIfNeeded(coord, tesselatorOutput, j);
            assert(snippedIndices.size() % 3 == 0);
            for (int k = 0; k < snippedIndices.size(); k += 3) {
              mCoordIndices.append(snippedIndices[k][0]);
              mCoordIndices.append(snippedIndices[k + 1][0]);
              mCoordIndices.append(snippedIndices[k + 2][0]);
              if (mNormalsValid) {
                mTmpNormalIndices.append(snippedIndices[k][1]);
                mTmpNormalIndices.append(snippedIndices[k + 1][1]);
                mTmpNormalIndices.append(snippedIndices[k + 2][1]);
              }
              if (mTextureCoordinatesValid) {
                mTmpTexIndices.append(snippedIndices[k][2]);
                mTmpTexIndices.append(snippedIndices[k + 1][2]);
                mTmpTexIndices.append(snippedIndices[k + 2][2]);
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
          IndicesString.append(QString::number(currentFaceIndices[j][0]));
        }
        IndicesString.append("]");
        mWarnings.append(QObject::tr("Out-of-range index in: %1.").arg(IndicesString));
      }

      currentFaceIndices.clear();
    }
    // add a coordIndex to the currentFace
    else
      currentFaceIndices.append(QVector<int>() << id << (mNormalsValid ? normalIndex->item(i) : 0)
                                               << (mTextureCoordinatesValid ? texCoordIndex->item(i) : 0));
  }

  assert(mCoordIndices.size() == mTmpNormalIndices.size() || mTmpNormalIndices.size() == 0);
  assert(mCoordIndices.size() == mTmpTexIndices.size() || mTmpTexIndices.size() == 0);
  assert(mCoordIndices.size() % 3 == 0);
}

QList<QVector<int>> WbTriangleMesh::cutTriangleIfNeeded(const WbMFVector3 *coord, const QList<QVector<int>> &tesselatedPolygon,
                                                        const int triangleIndex) {
  QList<QVector<int>> results;

  // find the three vertices of this triangle from the tesselated polygon
  const int firstVertexIndex = tesselatedPolygon[triangleIndex][0];
  const int secondVertexIndex = tesselatedPolygon[triangleIndex + 1][0];
  const int thirdVertexIndex = tesselatedPolygon[triangleIndex + 2][0];

  const int firstNormalIndex = tesselatedPolygon[triangleIndex][1];
  const int secondNormalIndex = tesselatedPolygon[triangleIndex + 1][1];
  const int thirdNormalIndex = tesselatedPolygon[triangleIndex + 2][1];

  const int firstTexCoordIndex = tesselatedPolygon[triangleIndex][2];
  const int secondTexCoordIndex = tesselatedPolygon[triangleIndex + 1][2];
  const int thirdTexCoordIndex = tesselatedPolygon[triangleIndex + 2][2];

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
    if (tesselatedPolygon[i][0] == firstVertexIndex || tesselatedPolygon[i][0] == secondVertexIndex ||
        tesselatedPolygon[i][0] == thirdVertexIndex)
      continue;
    // skip vertices we've already checked
    else if (checkedIndices.value(tesselatedPolygon[i][0]))
      continue;

    // case 1: vertex is on the first edge of the triangle
    else if (coord->item(tesselatedPolygon[i][0]).isOnEdgeBetweenVertices(firstEdgeStart, firstEdgeEnd)) {
      // first triangle
      results.append(QVector<int>() << firstVertexIndex << firstNormalIndex << firstTexCoordIndex);
      results.append(QVector<int>() << tesselatedPolygon[i][0] << tesselatedPolygon[i][1] << tesselatedPolygon[i][2]);
      results.append(QVector<int>() << thirdVertexIndex << thirdNormalIndex << thirdTexCoordIndex);
      // second triangle
      results.append(QVector<int>() << tesselatedPolygon[i][0] << tesselatedPolygon[i][1] << tesselatedPolygon[i][2]);
      results.append(QVector<int>() << secondVertexIndex << secondNormalIndex << secondTexCoordIndex);
      results.append(QVector<int>() << thirdVertexIndex << thirdNormalIndex << thirdTexCoordIndex);
    }
    // case 2: vertex is on the second edge of the triangle
    else if (coord->item(tesselatedPolygon[i][0]).isOnEdgeBetweenVertices(secondEdgeStart, secondEdgeEnd)) {
      // first triangle
      results.append(QVector<int>() << secondVertexIndex << secondNormalIndex << secondTexCoordIndex);
      results.append(QVector<int>() << tesselatedPolygon[i][0] << tesselatedPolygon[i][1] << tesselatedPolygon[i][2]);
      results.append(QVector<int>() << firstVertexIndex << firstNormalIndex << firstTexCoordIndex);
      // second triangle
      results.append(QVector<int>() << tesselatedPolygon[i][0] << tesselatedPolygon[i][1] << tesselatedPolygon[i][2]);
      results.append(QVector<int>() << thirdVertexIndex << thirdNormalIndex << thirdTexCoordIndex);
      results.append(QVector<int>() << firstVertexIndex << firstNormalIndex << firstTexCoordIndex);
    }
    // case 3: vertex is on the third edge of the triangle
    else if (coord->item(tesselatedPolygon[i][0]).isOnEdgeBetweenVertices(thirdEdgeStart, thirdEdgeEnd)) {
      // first triangle
      results.append(QVector<int>() << thirdVertexIndex << thirdNormalIndex << thirdTexCoordIndex);
      results.append(QVector<int>() << tesselatedPolygon[i][0] << tesselatedPolygon[i][1] << tesselatedPolygon[i][2]);
      results.append(QVector<int>() << secondVertexIndex << secondNormalIndex << secondTexCoordIndex);
      // second triangle
      results.append(QVector<int>() << tesselatedPolygon[i][0] << tesselatedPolygon[i][1] << tesselatedPolygon[i][2]);
      results.append(QVector<int>() << firstVertexIndex << firstNormalIndex << firstTexCoordIndex);
      results.append(QVector<int>() << secondVertexIndex << secondNormalIndex << secondTexCoordIndex);
    }

    // add this vertex to the list of those already checked
    checkedIndices.insert(tesselatedPolygon[i][0], true);
  }
  //  default - no need to cut the triangle, return it as-was
  if (results.isEmpty()) {
    results.append(QVector<int>() << firstVertexIndex << firstNormalIndex << firstTexCoordIndex);
    results.append(QVector<int>() << secondVertexIndex << secondNormalIndex << secondTexCoordIndex);
    results.append(QVector<int>() << thirdVertexIndex << thirdNormalIndex << thirdTexCoordIndex);
  }

  return results;
}

// populate mTmpTriangleNormals from coord and mCoordIndices
QString WbTriangleMesh::tmpNormalsPass(const WbMFVector3 *coord, const WbMFVector3 *normal) {
  assert(mNTriangles == mCoordIndices.size() / 3);
  assert(mCoordIndices.size() % 3 == 0);

  if (mNormalsValid && mNormalPerVertex)
    return "";  // normal are already defined per vertex

  // 1. compute normals per triangle
  for (int i = 0; i < mNTriangles; ++i) {
    if (mNormalsValid)
      mTmpTriangleNormals.append(normal->item(i).normalized());
    else {
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
      WbVector3 n(v1.cross(v2));
      const double length = n.length();
      if (length == 0.0)
        return QObject::tr("Null normal for face %1 %2 %3.\nThis can be caused by duplicate vertices in your mesh. "
                           "Try to open your model in a 3D modeling software, remove any duplicate vertices, and re-import the "
                           "model in Webots.")
          .arg(indexA)
          .arg(indexB)
          .arg(indexC);
      n /= length;
      mTmpTriangleNormals.append(n);
    }
  }

  assert(mTmpTriangleNormals.size() == mNTriangles);

  // 2. compute the map coordIndex->triangleIndex
  for (int t = 0; t < mNTriangles; ++t) {
    const int k = 3 * t;
    int j = mCoordIndices[k];
    mTmpVertexToTriangle.insert(j, t);
    j = mCoordIndices[k + 1];
    mTmpVertexToTriangle.insert(j, t);
    j = mCoordIndices[k + 2];
    mTmpVertexToTriangle.insert(j, t);
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

  int i = 0;
  WbVector3 vertices[3];
  for (int t = 0; t < mNTriangles; ++t) {  // foreach triangle
    vertices[0] = coord->item(mCoordIndices[i]);
    vertices[1] = coord->item(mCoordIndices[i + 1]);
    vertices[2] = coord->item(mCoordIndices[i + 2]);

    // compute face center and normal
    const WbVector3 edge1(vertices[1] - vertices[0]);
    const WbVector3 edge2(vertices[2] - vertices[0]);
    WbVector3 normalVector(edge1.cross(edge2));
    normalVector.normalize();
    const WbVector3 origin((vertices[0] + vertices[1] + vertices[2]) / 3.0);

    // compute intersection with the bounding box
    const WbRay faceNormal(origin, normalVector);
    double tmin, tmax;
    const std::pair<bool, double> result = faceNormal.intersects(minBound, maxBound, tmin, tmax);
    assert(result.first);
    const int faceIndex = WbBox::findIntersectedFace(minBound, maxBound, origin + result.second * normalVector);

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

    i += 3;
  }
}

// populate mIndices, mCoordinates, mTextureCoordinates and mNormals
void WbTriangleMesh::finalPass(const WbMFVector3 *coord, const WbMFVector3 *normal, const WbMFVector2 *texCoord,
                               double creaseAngle) {
  assert(coord && coord->size() > 0);
  assert(mTmpTriangleNormals.size() == mNTriangles || (mNormalsValid && mNormalPerVertex));
  assert(mNTriangles == mCoordIndices.size() / 3);
  assert(mCoordIndices.size() % 3 == 0);
  assert(mCoordIndices.size() == mTmpTexIndices.size() || mTmpTexIndices.size() == 0);
  const int texCoordSize = texCoord ? texCoord->size() : 0;
  const int normalSize = normal ? normal->size() : 0;
  const int coordSize = coord->size();

  // populate the vertex array
  WbVector3 vertexVector = coord->item(0);
  mMax[X] = vertexVector.x();
  mMax[Y] = vertexVector.y();
  mMax[Z] = vertexVector.z();
  mMin[X] = mMax[X];
  mMin[Y] = mMax[Y];
  mMin[Z] = mMax[Z];
  for (int i = 0; i < coordSize; ++i) {
    vertexVector = coord->item(i);

    const double x = vertexVector.x();
    if (mMax[X] < x)
      mMax[X] = x;
    else if (mMin[X] > x)
      mMin[X] = x;

    const double y = vertexVector.y();
    if (mMax[Y] < y)
      mMax[Y] = y;
    else if (mMin[Y] > y)
      mMin[Y] = y;

    const double z = vertexVector.z();
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
      const int i = k + v;
      const int indexCoord = mCoordIndices[i];

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
        for (int j = 0; j < ltSize; ++j) {
          const int linkedTriangleIndex = linkedTriangles.at(j);
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
        const int indexNormal = mTmpNormalIndices[i];
        if (indexNormal >= 0 && indexNormal < normalSize) {
          const WbVector3 nor(normal->item(indexNormal));
          mNormals.append(nor.x());
          mNormals.append(nor.y());
          mNormals.append(nor.z());
          mIsNormalCreased.append(false);
        }
      }

      if (mTextureCoordinatesValid) {
        const int indexTex = mTmpTexIndices[i];
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
    const int i = coordIndexIt.next();
    if (i != -1 && !coordIndexIt.hasNext())
      ++currentFaceIndicesCounter;

    if (i == -1 || !coordIndexIt.hasNext()) {
      int nCurrentFaceTriangle = qMax(0, currentFaceIndicesCounter - 2);
      nTriangles += nCurrentFaceTriangle;
      currentFaceIndicesCounter = 0;
    } else
      ++currentFaceIndicesCounter;
  }
  return nTriangles;
}

void WbTriangleMesh::updateScaledCoordinates(double x, double y, double z) {
  const int n = mCoordinates.size();
  assert(n % 3 == 0);
  mScaledCoordinates.resize(n);
  int i = 0;
  while (i < n) {
    mScaledCoordinates[i] = x * mCoordinates.at(i);
    ++i;
    mScaledCoordinates[i] = y * mCoordinates.at(i);
    ++i;
    mScaledCoordinates[i] = z * mCoordinates.at(i);
    ++i;
  }
}
