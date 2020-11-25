import {WbGeometry} from "./WbGeometry.js"

class WbTriangleMeshGeometry extends WbGeometry {
  constructor(id) {
    super(id);
  }

  createWrenObjects() {
    if (!mTriangleMeshError.isEmpty())
      console.error(mTriangleMeshError);

    super.createWrenObjects();

    buildWrenMesh(false);
  }
}

export {WbTriangleMeshGeometry}
