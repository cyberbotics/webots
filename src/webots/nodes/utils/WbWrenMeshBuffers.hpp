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

#ifndef WB_WREN_MESH_BUFFERS_HPP
#define WB_WREN_MESH_BUFFERS_HPP

//
// Description: utility class for creating and managing manual meshes.
//              Normals, vertices color and texture coordinates are
//              defined per vertex.
//

#include <stdint.h>
#include <QtCore/QVector>

class WbMatrix4;
class WbMatrix3;

class WbWrenMeshBuffers {
public:
  WbWrenMeshBuffers(int verticesCount, int indicesCount, int texCoordSetsCount, int colorBufferSize);
  WbWrenMeshBuffers();
  ~WbWrenMeshBuffers();

  void clear();
  void resetIndices();

  void setVertexBuffer(float *buffer);
  void setNormalBuffer(float *buffer);

  float *vertexBuffer() { return mVertexBuffer; }
  float *normalBuffer() { return mNormalBuffer; }
  uint32_t *colorBuffer() { return mColorBuffer; }
  float *texCoordBuffer() { return mTexCoordBuffer; }
  float *unwrappedTexCoordBuffer() { return mUnwrappedTexCoordsBuffer; }
  uint32_t *indexBuffer() { return mIndexBuffer; }

  int vertexIndex() const { return mVertexIndex; }
  int index() const { return mIndex; }
  int texCoordSetsCount() const { return mTexCoordSetsCount; }
  int colorIndex() const { return mColorIndex; }
  int colorBufferSize() const { return mColorBufferSize; }
  int verticesCount() const { return mVerticesCount; }
  int indicesCount() const { return mIndicesCount; }

  void setVertexIndex(int index) { mVertexIndex = index; }
  void setIndex(int index) { mIndex = index; }
  void setColorIndex(int colorIndex) { mColorIndex = colorIndex; }

  static void writeCoordinates(double x, double y, double z, const WbMatrix4 &m, float *buffer, int index);
  static void writeNormal(double x, double y, double z, const WbMatrix3 &m, float *buffer, int index);

private:
  float *mVertexBuffer;
  float *mNormalBuffer;
  uint32_t *mColorBuffer;
  float *mTexCoordBuffer;
  float *mUnwrappedTexCoordsBuffer;
  uint32_t *mIndexBuffer;

  int mVertexIndex;
  int mIndex;
  int mColorIndex;
  int mColorBufferSize;
  int mTexCoordSetsCount;
  int mVerticesCount;
  int mIndicesCount;

  bool mIsExternalVertexBuffer;
  bool mIsExternalNormalBuffer;

  void deleteVertexBuffer();
  void deleteNormalBuffer();
  void deleteTexCoordBuffer();
  void deleteColorBuffer();
  void deleteIndexBuffer();
  void resetAll(int verticesCount, int indicesCount, int texCoordSetsCount, int colorBufferSize);
};

#endif
