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

#ifndef WB_TRIANGLE_MESH_HPP
#define WB_TRIANGLE_MESH_HPP

//
// Description: helper class
//

#include <QtCore/QList>
#include <QtCore/QMultiHash>
#include <QtCore/QString>
#include <QtCore/QStringList>
#include <QtCore/QVarLengthArray>

#include "WbVector3.hpp"

class WbMFInt;
class WbMFVector2;
class WbMFVector3;

class WbTriangleMesh {
public:
  WbTriangleMesh();
  virtual ~WbTriangleMesh();

  // to be initialized from a WbIndexedFaceSet
  QString init(const WbMFVector3 *coord, const WbMFInt *coordIndex, const WbMFVector3 *normal, const WbMFInt *normalIndex,
               const WbMFVector2 *texCoord, const WbMFInt *texCoordIndex, double creaseAngle, bool normalPerVertex);
  // to be initialized from a WbMesh
  QString init(const double *coord, const double *normal, const double *texCoord, const unsigned int *index,
               int numberOfVertices, int indexSize);

  void cleanup();

  bool isValid() const { return mValid; }
  bool areTextureCoordinatesValid() const { return mTextureCoordinatesValid; }

  int numberOfTriangles() const { return mNTriangles; }
  int numberOfVertices() const { return mCoordinates.size() / 3; }

  static int index(int triangle, int vertex) { return 3 * triangle + vertex; }
  double vertex(int triangle, int vertex, int component) const {
    return mCoordinates[coordinateIndex(triangle, vertex, component)];
  }
  double normal(int triangle, int vertex, int component) const { return mNormals[3 * index(triangle, vertex) + component]; }
  bool isNormalCreased(int triangle, int vertex) const { return mIsNormalCreased[index(triangle, vertex)]; }
  double textureCoordinate(int triangle, int vertex, int component) const {
    return mTextureCoordinates[2 * index(triangle, vertex) + component];
  }
  double nonRecursiveTextureCoordinate(int triangle, int vertex, int component) const {
    return mNonRecursiveTextureCoordinates[2 * index(triangle, vertex) + component];
  }

  const int *indicesData() const { return mCoordIndices.data(); }
  const double *coordinatesData() const { return mCoordinates.data(); }
  const double *normalsData() const { return mNormals.data(); }
  const double *textureCoordinatesData() const { return mTextureCoordinates.data(); }

  const QStringList &warnings() const { return mWarnings; }
  const double *scaledCoordinatesData() const { return mScaledCoordinates.data(); }
  void updateScaledCoordinates(double x, double y, double z);
  double scaledVertex(int triangle, int vertex, int component) const {
    return mScaledCoordinates[coordinateIndex(triangle, vertex, component)];
  }
  bool areScaledCoordinatesEmpty() const { return mScaledCoordinates.isEmpty(); }

  double max(int coordinate) const { return mMax[coordinate]; }
  double min(int coordinate) const { return mMin[coordinate]; }

private:
  static int estimateNumberOfTriangles(const WbMFInt *coordIndex);
  int coordinateIndex(int triangle, int vertex, int component) const {
    return 3 * mCoordIndices[index(triangle, vertex)] + component;
  }

  void cleanupTmpArrays();

  bool mValid;
  bool mTextureCoordinatesValid;
  bool mNormalsValid;
  bool mNormalPerVertex;
  int mNTriangles;

  // populate mTmpCoordIndices and mTmpTexIndices
  void indicesPass(const WbMFVector3 *coord, const WbMFInt *coordIndex, const WbMFInt *normalIndex,
                   const WbMFInt *texCoordIndex);
  // contains triplet of coordinate indices (valid triangles)
  QVarLengthArray<int, 1> mCoordIndices;
  // contains doublet of normal indices (match with the mTmpNormalIndices order) or nothing
  QVarLengthArray<int, 1> mTmpNormalIndices;
  // contains doublet of texture indices (match with the mTmpCoordIndices order) or nothing
  QVarLengthArray<int, 1> mTmpTexIndices;
  // populate mTmpTriangleNormals and mTmpVertexNormals
  QString tmpNormalsPass(const WbMFVector3 *coord, const WbMFVector3 *normal);
  // contains normal vectors for each triangle (match with the mTmpCoordIndices order)
  QVarLengthArray<WbVector3, 1> mTmpTriangleNormals;
  // contains a map coordIndex->triangleIndex
  QMultiHash<int, int> mTmpVertexToTriangle;
  // populate mIndices, mCoordinates, mTextureCoordinates and mNormals
  void finalPass(const WbMFVector3 *coord, const WbMFVector3 *normal, const WbMFVector2 *texCoord, double creaseAngle);
  // populate mTextureCoordinates with default values
  void setDefaultTextureCoordinates(const WbMFVector3 *coord);
  // contains triplets representing the vertex coordinates (match with the mIndices order)
  QVarLengthArray<double, 1> mCoordinates;
  // contains triplets representing the vertex coordinates scaled by an absolute factor
  QVarLengthArray<double, 1> mScaledCoordinates;
  // contains doublet representing the texture coordinates (match with the mIndices order or default values)
  QVarLengthArray<double, 1> mTextureCoordinates;
  // contains doublet representing the non-recursive texture coordinates (match with the mIndices order) or nothing
  QVarLengthArray<double, 1> mNonRecursiveTextureCoordinates;
  // contains triplet representing the normals (either per triangle or per vertex) (match with the mIndices order)
  QVarLengthArray<double, 1> mNormals;
  QVarLengthArray<bool, 1> mIsNormalCreased;
  // improve tesselation problems by cutting bad triangles
  QList<QVector<int>> cutTriangleIfNeeded(const WbMFVector3 *coord, const QList<QVector<int>> &tesselatedPolygon,
                                          const int triangleIndex);
  QStringList mWarnings;

  enum { X = 0, Y = 1, Z = 2 };
  double mMax[3];
  double mMin[3];
};

#endif
