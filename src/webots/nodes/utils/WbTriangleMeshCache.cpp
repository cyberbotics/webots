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

#include "WbTriangleMeshCache.hpp"

#include "WbCoordinate.hpp"
#include "WbMFInt.hpp"
#include "WbNormal.hpp"
#include "WbSFBool.hpp"
#include "WbSFDouble.hpp"
#include "WbTextureCoordinate.hpp"
#include "WbTriangleMesh.hpp"
#include "WbTriangleMeshGeometry.hpp"

#include <cassert>
#include <cstdlib>
#include <functional>

namespace WbTriangleMeshCache {
  const highwayhash::HH_U64 SIPHASH_KEY[2] = {
    0x4242424242424242ull,
    0x4242424242424242ull,
  };

  uint64_t sipHash13c(const char *bytes, const int size) {
    return highwayhash::SipHash13(SIPHASH_KEY, bytes, size);
  }
  TriangleMeshInfo::TriangleMeshInfo() : mTriangleMesh(NULL), mNumUsers(0) {
  }
  TriangleMeshInfo::TriangleMeshInfo(WbTriangleMesh *triangleMesh) : mTriangleMesh(triangleMesh), mNumUsers(1) {
  }

  TriangleMeshGeometryKey::TriangleMeshGeometryKey() {
    mHash = 0;
  }
  TriangleMeshGeometryKey::TriangleMeshGeometryKey(WbTriangleMeshGeometry *triangleMeshGeometry) {
    set(triangleMeshGeometry);
  }

  void TriangleMeshGeometryKey::set(WbTriangleMeshGeometry *triangleMeshGeometry) {
    mHash = triangleMeshGeometry->computeHash();
  }

  bool TriangleMeshGeometryKey::operator==(const TriangleMeshGeometryKey &rhs) const {
    return mHash == rhs.mHash;
  }

  std::size_t TriangleMeshGeometryKeyHasher::operator()(const TriangleMeshGeometryKey &k) const {
    assert(sizeof(size_t) == sizeof(uint64_t));
    return static_cast<size_t>(k.mHash);
  }

  void useTriangleMesh(WbTriangleMeshGeometry *user) {
    if (user->getTriangleMeshMap().count(user->getMeshKey()) == 0 ||
        !user->getTriangleMeshMap()[user->getMeshKey()].mTriangleMesh->isValid())
      user->getTriangleMeshMap()[user->getMeshKey()] = user->createTriangleMesh();
    else
      ++user->getTriangleMeshMap().at(user->getMeshKey()).mNumUsers;

    user->setTriangleMesh(user->getTriangleMeshMap().at(user->getMeshKey()).mTriangleMesh);
    user->updateOdeData();
  }

  void releaseTriangleMesh(WbTriangleMeshGeometry *user) {
    if (user->getTriangleMeshMap().find(user->getMeshKey()) == user->getTriangleMeshMap().end())
      return;
    TriangleMeshInfo &triangleMeshInfo = user->getTriangleMeshMap().at(user->getMeshKey());
    if (--triangleMeshInfo.mNumUsers == 0) {
      delete triangleMeshInfo.mTriangleMesh;
      user->getTriangleMeshMap().erase(user->getMeshKey());
    } else
      assert(triangleMeshInfo.mNumUsers > 0);

    user->setTriangleMesh(NULL);
  }
}  // namespace WbTriangleMeshCache
