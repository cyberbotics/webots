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

#ifndef FILE_IMPORT_HPP
#define FILE_IMPORT_HPP

namespace wren {

  class StaticMesh;
  class DynamicMesh;
  class Skeleton;

  namespace fileImport {

    // Load a mesh from OBJ file
    // Returns false in case of failure
    bool importStaticMeshFromObj(const char *fileName, StaticMesh *mesh);

    const char *importRiggedMesh(const char *fileName, Skeleton **outputSkeleton, DynamicMesh ***outputMeshes,
                                 const char ***materialNames, int *count);
  };  // namespace fileImport

}  // namespace wren

#endif  // FILE_IMPORT_HPP
