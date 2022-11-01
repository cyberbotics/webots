// Copyright 1996-2022 Cyberbotics Ltd.
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

#include "FileImport.hpp"

#include <wren/file_import.h>

#include <assimp/postprocess.h>
#include <assimp/scene.h>
#include <assimp/Importer.hpp>

#include "Cache.hpp"
#include "DynamicMesh.hpp"
#include "Skeleton.hpp"
#include "SkeletonBone.hpp"
#include "StaticMesh.hpp"

#include <fstream>
#include <list>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

#ifdef _WIN32
#include <windows.h>
#endif

namespace wren {
  namespace fileImport {

    // Functions to split strings (https://stackoverflow.com/a/236803/4624559)
    template<typename Out> static void split(const std::string &s, char delim, Out result) {
      std::stringstream ss;
      ss.str(s);
      std::string item;
      while (std::getline(ss, item, delim))
        *(result++) = item;
    }

    static std::vector<std::string> split(const std::string &s, char delim) {
      std::vector<std::string> elems;
      split(s, delim, std::back_inserter(elems));
      return elems;
    }

    bool importStaticMeshFromObj(const char *fileName, StaticMesh **outputMesh) {
      const int len = strlen(fileName);
      const cache::Key key(cache::sipHash13c(fileName, len));

      StaticMesh *mesh;
      if (StaticMesh::createOrRetrieveFromCache(&mesh, key)) {
        *outputMesh = mesh;
        return true;
      }

#ifdef _WIN32  // handle UTF-8 paths on Windows
      wchar_t *wFileName = new wchar_t[len + 1];
      MultiByteToWideChar(CP_UTF8, 0, fileName, -1, wFileName, len + 1);
      std::ifstream infile(wFileName, std::ios::in | std::ios::binary);
      delete[] wFileName;
#else
      std::ifstream infile(fileName, std::ios::in | std::ios::binary);
#endif

      std::string line;
      glm::vec3 zero(0.0f, 0.0f, 0.0f);

      std::vector<glm::vec3> normals;
      std::vector<glm::vec3> normalsByVertex;
      std::vector<int> normalsCount;

      while (std::getline(infile, line)) {
        if (line.size() == 0)
          continue;

        std::vector<std::string> tokens = split(line, ' ');

        if (tokens[0] == "v") {
          if (tokens.size() < 4)
            return false;

          float coord[3] = {std::strtof(tokens[1].c_str(), NULL), std::strtof(tokens[2].c_str(), NULL),
                            std::strtof(tokens[3].c_str(), NULL)};
          mesh->addCoord(glm::make_vec3(coord));
          mesh->addTexCoord(zero);
          normalsByVertex.push_back(zero);
          normalsCount.push_back(0);
        } else if (tokens[0] == "vn") {
          if (tokens.size() < 4)
            return false;

          float coords[3] = {std::strtof(tokens[1].c_str(), NULL), std::strtof(tokens[2].c_str(), NULL),
                             std::strtof(tokens[3].c_str(), NULL)};

          normals.push_back(glm::make_vec3(coords));
        } else if (tokens[0] == "f") {
          if (tokens.size() < 4)
            return false;

          for (int i = 1; i < 4; ++i) {
            std::vector<std::string> faces = split(tokens[i], '/');
            int vertexIndex = std::stoi(faces[0]) - 1;
            mesh->addIndex(vertexIndex);

            int normalIndex = std::stoi(faces[2]) - 1;
            normalsByVertex[vertexIndex] += normals[normalIndex];
            normalsCount[vertexIndex]++;
          }
        } else
          continue;
      }

      for (size_t i = 0; i < normalsByVertex.size(); ++i)
        mesh->addNormal(normalsByVertex[i] / static_cast<float>(normalsCount[i]));

      // bounding volumes
      mesh->computeBoundingVolumes();

      mesh->setup();

      *outputMesh = mesh;
      return true;
    }

    static void applyTransforms(aiNode *node, aiMatrix4x4 parentTransform, std::unordered_map<std::string, size_t> &boneIndices,
                                std::vector<SkeletonBone *> &bones, std::vector<aiMatrix4x4> &matrices) {
      if (!node)
        return;

      aiMatrix4x4 globalTransform = parentTransform * node->mTransformation;
      std::string nodeName(node->mName.data);
      if (boneIndices.find(nodeName) != boneIndices.end()) {
        const size_t boneIndex = boneIndices[nodeName];
        SkeletonBone *bone = bones[boneIndex];
        aiMatrix4x4 finalTransform = matrices[boneIndex];
        aiVector3t<float> scaling, position;
        aiQuaternion rotation;
        finalTransform.Decompose(scaling, rotation, position);
        bone->setAbsolutePosition(glm::make_vec3(&position[0]));
        bone->setAbsoluteScale(glm::make_vec3(&scaling[0]));
        const float orientation[4] = {rotation.x, rotation.y, rotation.z, rotation.w};
        bone->setAbsoluteOrientation(glm::make_quat(&orientation[0]));
      }

      for (size_t i = 0; i < node->mNumChildren; ++i)
        applyTransforms(node->mChildren[i], globalTransform, boneIndices, bones, matrices);
    }

    const char *importRiggedMeshFromFile(const char *fileName, Skeleton **outputSkeleton, DynamicMesh ***outputMeshes,
                                         const char ***materialNames, int *count) {
      Assimp::Importer importer;

      const aiScene *scene = importer.ReadFile(
        fileName, aiProcess_ValidateDataStructure | aiProcess_Triangulate | aiProcess_JoinIdenticalVertices |
                    aiProcess_GenSmoothNormals | aiProcess_FindInvalidData | aiProcess_TransformUVCoords | aiProcess_FlipUVs);
      return importRiggedMesh(scene, outputSkeleton, outputMeshes, materialNames, count);
    }

    const char *importRiggedMeshFromMemory(const char *data, int size, const char *hint, Skeleton **outputSkeleton,
                                           DynamicMesh ***outputMeshes, const char ***materialNames, int *count) {
      Assimp::Importer importer;
      const aiScene *scene = importer.ReadFileFromMemory(
        data, size,
        aiProcess_ValidateDataStructure | aiProcess_Triangulate | aiProcess_JoinIdenticalVertices | aiProcess_GenSmoothNormals |
          aiProcess_FindInvalidData | aiProcess_TransformUVCoords | aiProcess_FlipUVs,
        hint);

      return importRiggedMesh(scene, outputSkeleton, outputMeshes, materialNames, count);
    }

    const char *importRiggedMesh(const aiScene *scene, Skeleton **outputSkeleton, DynamicMesh ***outputMeshes,
                                 const char ***materialNames, int *count) {
      if (!scene)
        return "Invalid data, please verify mesh file (bone weights, normals, ...).";
      else if (!scene->HasMeshes())
        return "File does not contain any mesh.";

      std::vector<DynamicMesh *> meshes;
      std::vector<const char *> materials;
      std::vector<aiMatrix4x4> offsetMatrices;
      std::unordered_map<std::string, size_t> boneIndices;

      aiMatrix4x4 globalInvTransform = scene->mRootNode->mTransformation;
      globalInvTransform.Inverse();

      Skeleton *skeleton = Skeleton::createSkeleton();

      std::list<aiNode *> queue;
      queue.push_back(scene->mRootNode);

      std::vector<SkeletonBone *> bones;
      while (!queue.empty()) {
        aiNode *node = queue.front();
        queue.pop_front();

        for (size_t i = 0; i < node->mNumMeshes; ++i) {
          const aiMesh *mesh = scene->mMeshes[node->mMeshes[i]];

          if (!(mesh->HasBones() && mesh->HasPositions() && mesh->HasFaces()))
            continue;

          DynamicMesh *dynamicMesh = DynamicMesh::createDynamicMesh(mesh->HasNormals(), mesh->HasTextureCoords(0), false);

          aiMatrix4x4 transform;
          aiNode *current = node->mParent;
          while (current != scene->mRootNode && current != NULL) {
            transform = transform * node->mTransformation;
            current = current->mParent;
          }
          transform = globalInvTransform * transform;

          glm::mat4 matrix = glm::transpose(glm::make_mat4(&transform[0][0]));

          // Build mesh
          for (size_t j = 0; j < mesh->mNumVertices; ++j) {
            dynamicMesh->addCoord(glm::vec3(matrix * glm::make_vec4(&mesh->mVertices[j][0])), true);

            if (mesh->HasNormals())
              dynamicMesh->addNormal(glm::vec3(matrix * glm::vec4(glm::make_vec3(&mesh->mNormals[j][0]), 0.0f)));

            if (mesh->HasTextureCoords(0))
              dynamicMesh->addTexCoord(glm::make_vec2(&mesh->mTextureCoords[0][j][0]));
          }

          for (size_t j = 0; j < mesh->mNumFaces; ++j) {
            const aiFace face = mesh->mFaces[j];
            assert(face.mNumIndices == 3);

            dynamicMesh->addIndex(face.mIndices[0]);
            dynamicMesh->addIndex(face.mIndices[1]);
            dynamicMesh->addIndex(face.mIndices[2]);
          }

          meshes.push_back(dynamicMesh);
          skeleton->addMesh(dynamicMesh);

          for (size_t j = 0; j < mesh->mNumBones; ++j) {
            const aiBone *bone = mesh->mBones[j];
            std::string boneName(bone->mName.data);

            SkeletonBone *wrenBone = skeleton->getBoneByName(boneName.c_str());
            if (!wrenBone) {
              wrenBone = SkeletonBone::createSkeletonBone(skeleton, boneName.c_str());
              boneIndices[boneName] = bones.size();
              bones.push_back(wrenBone);
              aiMatrix4x4 boneMatrix = bone->mOffsetMatrix;
              boneMatrix.Inverse();
              offsetMatrices.push_back(boneMatrix);
            }

            for (size_t k = 0; k < bone->mNumWeights; ++k) {
              const aiVertexWeight weight = bone->mWeights[k];
              skeleton->attachVertexToBone(dynamicMesh, weight.mVertexId, wrenBone, weight.mWeight);
            }
          }

          // Get material
          const unsigned int materialIndex = mesh->mMaterialIndex;
          aiMaterial *material = scene->mMaterials[materialIndex];
          aiString name;
          material->Get(AI_MATKEY_NAME, name);
          char *matName = new char[strlen(name.C_Str())];
          strcpy(matName, name.C_Str());
          materials.push_back(matName);
        }

        for (size_t i = 0; i < node->mNumChildren; ++i)
          queue.push_back(node->mChildren[i]);
      }

      // Bones hierarchy
      for (SkeletonBone *bone : bones) {
        aiNode *node = scene->mRootNode->FindNode(bone->name());
        assert(node != NULL);
        aiNode *current = node->mParent;
        while (current != NULL) {
          if (current->mName.length > 0) {
            SkeletonBone *parentBone = skeleton->getBoneByName(current->mName.C_Str());
            if (parentBone != NULL) {
              parentBone->attachChild(bone);
              break;
            }
          }
          current = current->mParent;
        }
      }

      // Setup bone transformations
      applyTransforms(scene->mRootNode, globalInvTransform, boneIndices, bones, offsetMatrices);

      *count = meshes.size();
      if (*count > 0) {
        skeleton->normalizeWeights();
        *outputSkeleton = skeleton;
        *outputMeshes = new DynamicMesh *[meshes.size()];
        for (size_t i = 0; i < meshes.size(); ++i)
          (*outputMeshes)[i] = meshes[i];

        *materialNames = new const char *[materials.size()];
        for (size_t i = 0; i < materials.size(); ++i)
          (*materialNames)[i] = materials[i];

        return NULL;
      } else
        return "File does not contain any valid rigged mesh.";
    }
  }  // namespace fileImport
}  // namespace wren

// C interface implementation
bool wr_import_static_mesh_from_obj(const char *fileName, WrStaticMesh **mesh) {
  return wren::fileImport::importStaticMeshFromObj(fileName, reinterpret_cast<wren::StaticMesh **>(mesh));
}

const char *wr_import_skeleton_from_file(const char *fileName, WrSkeleton **skeleton, WrDynamicMesh ***meshes,
                                         const char ***materials, int *count) {
  return wren::fileImport::importRiggedMeshFromFile(fileName, reinterpret_cast<wren::Skeleton **>(skeleton),
                                                    reinterpret_cast<wren::DynamicMesh ***>(meshes), materials, count);
}

const char *wr_import_skeleton_from_memory(const char *data, int size, const char *hint, WrSkeleton **skeleton,
                                           WrDynamicMesh ***meshes, const char ***materials, int *count) {
  return wren::fileImport::importRiggedMeshFromMemory(data, size, hint, reinterpret_cast<wren::Skeleton **>(skeleton),
                                                      reinterpret_cast<wren::DynamicMesh ***>(meshes), materials, count);
}
