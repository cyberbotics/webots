import {WbVector3} from "./WbVector3.js";

class WbTesselator {

  static tessBegin(type) {
    assert(type === libtess.primitiveType.GL_TRIANGLES);
  }

  // the glu tesselator calls this function in the right order to populate the
  // index list
  static tessVertexData(vertex, r) {
    /*console.log("vertex : ");
    console.log(vertex);
    console.log("r : ");
    console.log(r);*/
    r.push(new WbVector3(vertex[0], vertex[1], vertex[2]));
  }

  static tessEnd() {
  }
  // used to force triangle list (and disable strips and fans)
  static tessEdgeFlag(flag) {
  }

  static tessError(errorCode) {
    console.err('error callback');
    console.err('error number: ' + errorCode);
    WbTesselator.error = true;
  }

  static tesselate(indexes, vertices, results) {

   // don't reallocate the results list into the tesselator callback
   const indexSize = indexes.length;
   const s = indexSize - 2;
   const maxSize = (s > 0) ? 3 * s : 0;
   const vertexSize = vertices.length;

   assert(indexSize == vertexSize);

   let tesselator = new libtess.GluTesselator();
   assert(tesselator);
   //TODO: the callback functions
   tesselator.gluTessCallback(libtess.gluEnum.GLU_TESS_BEGIN, WbTesselator.tessBegin);
   tesselator.gluTessCallback(libtess.gluEnum.GLU_TESS_VERTEX_DATA, WbTesselator.tessVertexData);
   tesselator.gluTessCallback(libtess.gluEnum.GLU_TESS_END, WbTesselator.tessEnd);
   tesselator.gluTessCallback(libtess.gluEnum.GLU_TESS_EDGE_FLAG, WbTesselator.tessEdgeFlag);
   tesselator.gluTessCallback(libtess.gluEnum.GLU_TESS_ERROR, WbTesselator.tessError);
   tesselator.gluTessProperty(libtess.gluEnum.GLU_TESS_WINDING_RULE, libtess.windingRule.GLU_TESS_WINDING_POSITIVE);
   tesselator.gluTessBeginPolygon(results);
   tesselator.gluTessBeginContour();

   for (let i = 0; i < vertexSize; i++) {
     const currentVertex = vertices[i];
     const pos = [currentVertex.x, currentVertex.y, currentVertex.z];
     const data = [indexes[i].x, indexes[i].y, indexes[i].z];

     tesselator.gluTessVertex(pos, data);
   }

   tesselator.gluTessEndContour(tesselator);
   tesselator.gluTessEndPolygon(tesselator);

   tesselator.gluDeleteTess(tesselator);

   // make sure no allocation was done in the tesselator callback
   assert(results.length <= maxSize);

   // check that the data is consistent, otherwise delete the result list
   if (results.length % 3 !== 0 || WbTesselator.error) {
     console.error("Tessellation Error: the result of the tesselation is not a multiple of 3.");
     results = [];
   }
 }
}

WbTesselator.error = false;

export {WbTesselator}
