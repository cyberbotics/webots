import {WbTriangleMeshGeometry} from "./WbTriangleMeshGeometry.js"

class WbIndexedFaceSet extends WbTriangleMeshGeometry {
  constructor(id, isDefaultMapping, coordIndex, normalIndex, texCoordIndex, coord, texCoord, normal, creaseAngle, ccw, normalPerVertex) {
    super(id);
    this.isDefaultMapping = isDefaultMapping;

    this.coordIndex = coordIndex;
    this.normalIndex = normalIndex;
    this.texCoordIndex = texCoordIndex;

    this.coord = coord;
    this.texCoord = texCoord;
    this.normal = normal;

    this.creaseAngle = creaseAngle;
    this.ccw = ccw;
    this.normalPerVertex = normalPerVertex;
  }

  updateTriangleMesh() {
    this.triangleMesh.init(this.coord, this.coordIndex, this.normal, this.normalIndex, this.texCoord, this.texCoordIndex, this.creaseAngle, this.ccw, this.normalPerVertex);
  }
}

export {WbIndexedFaceSet}
