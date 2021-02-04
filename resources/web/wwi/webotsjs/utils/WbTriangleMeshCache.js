// Copyright 1996-2021 Cyberbotics Ltd.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

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
