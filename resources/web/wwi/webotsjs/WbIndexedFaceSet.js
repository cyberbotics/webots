import {WbTriangleMeshGeometry} from "./WbTriangleMeshGeometry.js"

class WbIndexedFaceSet extends WbTriangleMeshGeometry {
  constructor(id, isDefaultMapping, coordIndex, normalIndex, texCoordIndex, coord, texCoord, normal) {
    super(id);
    this.isDefaultMapping = isDefaultMapping;

    this.coordIndex = coordIndex;
    this.normalIndex = normalIndex;
    this.texCoordIndex = texCoordIndex;

    this.coord = coord;
    this.texCoord = texCoord;
    this.normal = normal;
  }
}

export {WbIndexedFaceSet}
