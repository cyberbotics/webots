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

#ifndef WB_TRIANGLE_MESH_CACHE_HPP
#define WB_TRIANGLE_MESH_CACHE_HPP

#include "sip_hash.hpp"

class WbTriangleMeshGeometry;
class WbTriangleMesh;

namespace WbTriangleMeshCache {
  extern const highwayhash::HH_U64 SIPHASH_KEY[2];

  template<class T> uint64_t sipHash13x(const T *bytes, const int size) {
    return highwayhash::SipHash13(SIPHASH_KEY, reinterpret_cast<const char *>(bytes), size * sizeof(T));
  }

  // TriangleMeshInfo is shared by all WbTriangleMeshGeometry instances requiring the same WbTriangleMesh.
  struct TriangleMeshInfo {
    TriangleMeshInfo();
    explicit TriangleMeshInfo(WbTriangleMesh *triangleMesh);

    WbTriangleMesh *mTriangleMesh;
    int mNumUsers;
  };

  // Key type for an instance of WbTriangleMeshGeometry. Instances can share a WbTriangleMesh if their keys compare equal.
  struct TriangleMeshGeometryKey {
    TriangleMeshGeometryKey();
    explicit TriangleMeshGeometryKey(WbTriangleMeshGeometry *triangleMeshGeometry);

    void set(WbTriangleMeshGeometry *triangleMeshGeometry);
    bool operator==(const TriangleMeshGeometryKey &rhs) const;

    uint64_t mHash;
  };

  // Key hashing function required by std::unordered_map
  struct TriangleMeshGeometryKeyHasher {
    std::size_t operator()(const TriangleMeshGeometryKey &k) const;
  };

  void useTriangleMesh(WbTriangleMeshGeometry *user);
  void releaseTriangleMesh(WbTriangleMeshGeometry *user);
}  // namespace WbTriangleMeshCache

#endif
