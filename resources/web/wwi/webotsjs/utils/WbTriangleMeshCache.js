class WbTriangleMeshCache {
  static releaseTriangleMesh(user) {
    let triangleMeshInfo = user.cTriangleMeshMap.at(user->getMeshKey());
    if (--triangleMeshInfo.mNumUsers == 0) {
      delete triangleMeshInfo.mTriangleMesh;
      user->getTriangleMeshMap().erase(user->getMeshKey());
    } else
      assert(triangleMeshInfo.mNumUsers >= 0);

    user->setTriangleMesh(NULL);
  }
}

export {WbTriangleMeshCache}
