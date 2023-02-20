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

#include "WbWrenMeshBuffers.hpp"

#include "WbGeometry.hpp"
#include "WbMatrix3.hpp"
#include "WbMatrix4.hpp"

WbWrenMeshBuffers::WbWrenMeshBuffers() :
  mVertexBuffer(NULL),
  mNormalBuffer(NULL),
  mColorBuffer(NULL),
  mTexCoordBuffer(NULL),
  mUnwrappedTexCoordsBuffer(NULL),
  mIndexBuffer(NULL),
  mVertexIndex(0),
  mIndex(0),
  mColorIndex(0),
  mColorBufferSize(0),
  mTexCoordSetsCount(0),
  mVerticesCount(0),
  mIndicesCount(0),
  mIsExternalVertexBuffer(false),
  mIsExternalNormalBuffer(false) {
}

WbWrenMeshBuffers::WbWrenMeshBuffers(int verticesCount, int indicesCount, int texCoordSetsCount, int colorBufferSize) :
  mVertexBuffer(NULL),
  mNormalBuffer(NULL),
  mColorBuffer(NULL),
  mTexCoordBuffer(NULL),
  mUnwrappedTexCoordsBuffer(NULL),
  mIndexBuffer(NULL),
  mVertexIndex(0),
  mIndex(0),
  mColorIndex(0),
  mTexCoordSetsCount(texCoordSetsCount),
  mVerticesCount(verticesCount),
  mIndicesCount(indicesCount),
  mIsExternalVertexBuffer(false),
  mIsExternalNormalBuffer(false) {
  resetAll(verticesCount, indicesCount, mTexCoordSetsCount, colorBufferSize);
}

WbWrenMeshBuffers::~WbWrenMeshBuffers() {
  clear();
}

void WbWrenMeshBuffers::clear() {
  deleteVertexBuffer();
  deleteNormalBuffer();
  deleteColorBuffer();
  deleteTexCoordBuffer();
  deleteIndexBuffer();
}

void WbWrenMeshBuffers::deleteVertexBuffer() {
  if (!mIsExternalVertexBuffer)
    delete[] mVertexBuffer;
  mVertexBuffer = NULL;
  mVertexIndex = 0;
  mIsExternalVertexBuffer = false;
}

void WbWrenMeshBuffers::deleteNormalBuffer() {
  if (!mIsExternalNormalBuffer)
    delete[] mNormalBuffer;
  mNormalBuffer = NULL;
  mIsExternalNormalBuffer = false;
}

void WbWrenMeshBuffers::deleteColorBuffer() {
  delete[] mColorBuffer;
  mColorBuffer = NULL;
}

void WbWrenMeshBuffers::deleteTexCoordBuffer() {
  delete[] mTexCoordBuffer;
  delete[] mUnwrappedTexCoordsBuffer;
  mTexCoordBuffer = NULL;
  mUnwrappedTexCoordsBuffer = NULL;
}

void WbWrenMeshBuffers::deleteIndexBuffer() {
  delete[] mIndexBuffer;
  mIndexBuffer = NULL;
  mIndex = 0;
}

void WbWrenMeshBuffers::resetAll(int verticesCount, int indicesCount, int texCoordSetsCount, int colorBufferSize) {
  clear();

  mVerticesCount = verticesCount;
  mIndicesCount = indicesCount;
  mTexCoordSetsCount = texCoordSetsCount;
  mVertexBuffer = new float[verticesCount * 3];
  mNormalBuffer = new float[verticesCount * 3];
  if (texCoordSetsCount > 0) {
    mTexCoordBuffer = new float[verticesCount * texCoordSetsCount * 2];
    mUnwrappedTexCoordsBuffer = new float[verticesCount * texCoordSetsCount * 2];
  }
  mIndexBuffer = new uint32_t[indicesCount];
  mColorBufferSize = colorBufferSize;
  if (colorBufferSize > 0)
    mColorBuffer = new uint32_t[colorBufferSize];
}

void WbWrenMeshBuffers::resetIndices() {
  mVertexIndex = 0;
  mColorIndex = 0;
  mIndex = 0;
}

void WbWrenMeshBuffers::setVertexBuffer(float *buffer) {
  if (!mIsExternalVertexBuffer)
    delete[] mVertexBuffer;
  mVertexBuffer = buffer;
  mIsExternalVertexBuffer = true;
}

void WbWrenMeshBuffers::setNormalBuffer(float *buffer) {
  if (!mIsExternalNormalBuffer)
    delete[] mNormalBuffer;
  mNormalBuffer = buffer;
  mIsExternalNormalBuffer = true;
}

void WbWrenMeshBuffers::writeCoordinates(double x, double y, double z, const WbMatrix4 &m, float *buffer, int index) {
  WbVector4 result = m * WbVector4(x, y, z, 1.0);
  buffer[index] = result.x();
  buffer[index + 1] = result.y();
  buffer[index + 2] = result.z();
}

void WbWrenMeshBuffers::writeNormal(double x, double y, double z, const WbMatrix3 &m, float *buffer, int index) {
  WbVector3 result = m * WbVector3(x, y, z);
  buffer[index] = result.x();
  buffer[index + 1] = result.y();
  buffer[index + 2] = result.z();
}
