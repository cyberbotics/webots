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

#include "WbTesselator.hpp"

#include "WbVector3.hpp"

#include <cassert>
#include <cstdlib>

#ifdef __APPLE__
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#include <OpenGL/glu.h>
#else
#include <GL/glu.h>
#endif

#include <QtCore/QObject>

#ifdef _WIN32
#define GLU_function_pointer GLvoid(APIENTRY *)()
#endif

#ifdef __APPLE__
#if MAC_OS_X_VERSION_MAX_ALLOWED <= MAC_OS_X_VERSION_10_4
#define GLU_function_pointer GLvoid (*)(...)
#else
typedef GLvoid (*GLU_function_pointer)();
#endif
#endif

#ifdef __linux__
#define GLU_function_pointer GLvoid (*)()
#endif

static QString errorString;

// GLU tesselator callback functions
extern "C" {
// data structure to communicate with the tesselator
struct TesselatorData {
  // must start with 3 GLdouble in order to work with the tesselator
  GLdouble pos[3];

  // extra data start from here
  int coordIndex;
  int normalIndex;
  int texIndex;
};

static void tessBegin(GLenum type);
static void tessVertexData(void *vertex, void *r);
static void tessEnd();
static void tessEdgeFlag(GLboolean flag);
static void tessError(GLenum errorCode);
}

static void tessBegin(GLenum type) {
  assert(type == GL_TRIANGLES);
}

static void tessEnd() {
}

// the glu tesselator calls this function in the right order to populate the
// index list
static void tessVertexData(void *vertex, void *r) {
  QList<QVector<int>> *results = (QList<QVector<int>> *)r;
  TesselatorData *tesselatorData = static_cast<TesselatorData *>(vertex);
  results->append(QVector<int>() << tesselatorData->coordIndex << tesselatorData->normalIndex << tesselatorData->texIndex);
}

// used to force triangle list (and disable strips and fans)
static void tessEdgeFlag(GLboolean flag) {
}

static void tessError(GLenum errorCode) {
  if (errorCode == GLU_TESS_NEED_COMBINE_CALLBACK)
    errorString = QObject::tr("Tessellation Error: the contour of a face must not self-intersect.");
  else
    errorString = QObject::tr("Tessellation Error (%1): \"%2\".")
                    .arg((int)errorCode)
                    .arg(reinterpret_cast<const char *>(gluErrorString(errorCode)));
}

QString WbTesselator::tesselate(const QList<QVector<int>> &indexes, const QList<WbVector3> &vertices,
                                QList<QVector<int>> &results) {
  results.clear();

  // don't reallocate the results list into the tesselator callback
  const int indexSize = indexes.size();
  const int s = indexSize - 2;
  const int maxSize = (s > 0) ? 3 * s : 0;
  results.reserve(maxSize);

  errorString = "";

  const int vertexSize = vertices.size();

  assert(indexSize == vertexSize);

  GLUtesselator *tesselator = gluNewTess();
  assert(tesselator);

  gluTessCallback(tesselator, GLU_TESS_BEGIN, (GLU_function_pointer)&tessBegin);
  gluTessCallback(tesselator, GLU_TESS_VERTEX_DATA, (GLU_function_pointer)&tessVertexData);
  gluTessCallback(tesselator, GLU_TESS_END, (GLU_function_pointer)&tessEnd);
  gluTessCallback(tesselator, GLU_TESS_EDGE_FLAG, (GLU_function_pointer)&tessEdgeFlag);
  gluTessCallback(tesselator, GLU_TESS_ERROR, (GLU_function_pointer)&tessError);
  gluTessProperty(tesselator, GLU_TESS_WINDING_RULE, GLU_TESS_WINDING_POSITIVE);
  gluTessBeginPolygon(tesselator, &results);
  gluTessBeginContour(tesselator);

  TesselatorData *tesselatorData = static_cast<TesselatorData *>(malloc(sizeof(TesselatorData) * vertexSize));

  for (int i = 0; i < vertexSize; i++) {
    const WbVector3 &currentVertex = vertices[i];

    TesselatorData *data = &tesselatorData[i];
    data->pos[0] = (GLdouble)currentVertex.x();
    data->pos[1] = (GLdouble)currentVertex.y();
    data->pos[2] = (GLdouble)currentVertex.z();
    data->coordIndex = indexes[i][0];
    data->normalIndex = indexes[i][1];
    data->texIndex = indexes[i][2];

    gluTessVertex(tesselator, data->pos, data);
  }

  gluTessEndContour(tesselator);
  gluTessEndPolygon(tesselator);

  gluDeleteTess(tesselator);

  // make sure no allocation was done in the tesselator callback
  assert(results.size() <= maxSize);

  free(tesselatorData);

  // check that the data is consistent, otherwise delete the result list
  if (results.size() % 3 != 0)
    errorString = QObject::tr("Tessellation Error: the result of the tesselation is not a multiple of 3.");

  if (!errorString.isEmpty())
    results.clear();

  return errorString;
}

#ifdef __APPLE__
#pragma GCC diagnostic pop
#endif
