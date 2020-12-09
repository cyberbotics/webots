class WbTesselator {
  static tesselate(indexes, vertices, results) {
    results = [];

   // don't reallocate the results list into the tesselator callback
   const indexSize = indexes.length;
   const s = indexSize - 2;
   const maxSize = (s > 0) ? 3 * s : 0;

   const int vertexSize = vertices.size();

   assert(indexSize == vertexSize);

   let tesselator = new libtess.GluTesselator();
   assert(tesselator);
   //TODO: the callback functions
   tesselator.gluTessCallback(GLU_TESS_BEGIN, (GLU_function_pointer)&tessBegin);
   tesselator.gluTessCallback(GLU_TESS_VERTEX_DATA, (GLU_function_pointer)&tessVertexData);
   tesselator.gluTessCallback(GLU_TESS_END, (GLU_function_pointer)&tessEnd);
   tesselator.gluTessCallback(GLU_TESS_EDGE_FLAG, (GLU_function_pointer)&tessEdgeFlag);
   tesselator.gluTessCallback(GLU_TESS_ERROR, (GLU_function_pointer)&tessError);
   tesselator.gluTessProperty(GLU_TESS_WINDING_RULE, GLU_TESS_WINDING_POSITIVE);
   tesselator.gluTessBeginPolygon(&results);
   tesselator.gluTessBeginContour();

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
}
export {WbTesselator}
