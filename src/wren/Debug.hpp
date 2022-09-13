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

#ifndef DEBUG_HPP
#define DEBUG_HPP

// Uncoment to activate the Wren log:
// #define WREN_DEBUG

#include "Camera.hpp"
#include "Constants.hpp"
#include "FrameBuffer.hpp"
#include "LightNode.hpp"
#include "PbrMaterial.hpp"
#include "PhongMaterial.hpp"
#include "PostProcessingEffect.hpp"
#include "Primitive.hpp"
#include "Renderable.hpp"
#include "Scene.hpp"
#include "Skeleton.hpp"
#include "StaticMesh.hpp"
#include "Texture2d.hpp"
#include "Transform.hpp"

#include <iostream>

namespace wren {

#ifndef WREN_DEBUG
  inline void PhongMaterial::printCacheContents() {}
  inline void PbrMaterial::printCacheContents() {}
  inline void StaticMesh::printCacheContents() {}
  inline void Texture2d::printCacheContents() {}
  inline void PostProcessingEffect::printPasses() const { assert(this); }
#else
  inline void PhongMaterial::printCacheContents() {
    std::cerr << "PhongMaterial cache, number of entries: " << PhongMaterial::cCache.size() << std::endl;
    for (const auto &data : PhongMaterial::cCache) {
      std::cerr << "Hash: " << data.first.mHash << ", users: " << data.second.mNumUsers << std::endl;
    }
  }

  inline void PbrMaterial::printCacheContents() {
    std::cerr << "PbrMaterial cache, number of entries: " << PbrMaterial::cCache.size() << std::endl;
    for (const auto &data : PbrMaterial::cCache) {
      std::cerr << "Hash: " << data.first.mHash << ", users: " << data.second.mNumUsers << std::endl;
    }
  }

  inline void StaticMesh::printCacheContents() {
    std::cerr << "Mesh cache, number of entries: " << StaticMesh::cCache.size() << std::endl;
    for (const auto &data : StaticMesh::cCache) {
      std::cerr << "Hash: " << data.first.mHash << ", users: " << data.second.mNumUsers << std::endl;
    }
  }

  inline void Texture2d::printCacheContents() {
    std::cerr << "Texture2d cache, number of entries: " << Texture2d::cCache.size() << std::endl;
    for (const auto &data : Texture2d::cCache) {
      std::cerr << "Hash: " << data.first.mHash << ", users: " << data.second.mNumUsers << std::endl;
    }
  }
  inline void PostProcessingEffect::printPasses() const {
    std::cerr << "PostProcessingEffect: number of passes = " << mPasses.size() << std::endl;
    for (size_t i = 0; i < mPasses.size(); ++i) {
      std::cerr << "  Pass " << i << ": " << std::endl;
      std::cerr << "    FrameBuffer: " << mPasses[i]->frameBuffer() << std::endl;
      std::cerr << "      width: " << mPasses[i]->frameBuffer()->width() << std::endl;
      std::cerr << "      height: " << mPasses[i]->frameBuffer()->height() << std::endl;
      std::cerr << "      depth texture: " << mPasses[i]->frameBuffer()->depthTexture() << std::endl;
      std::cerr << "    Program: " << mPasses[i]->program() << std::endl;
      std::cerr << "    Input textures: " << std::endl;
      for (size_t j = 0; j < mPasses[i]->inputTextures().size(); ++j)
        std::cerr << "      Texture " << j << ": " << mPasses[i]->inputTextures()[j] << std::endl;
      std::cerr << "    Output textures: " << std::endl;
      for (size_t j = 0; j < mPasses[i]->frameBuffer()->outputTextures().size(); ++j)
        std::cerr << "      Texture " << j << ": " << mPasses[i]->frameBuffer()->outputTexture(j) << std::endl;
    }
  }
#endif

  namespace debug {

#ifndef WREN_DEBUG
#define DEBUG(x) \
  do {           \
  } while (0)
#define DEBUGONE(x) \
  do {              \
  } while (0)
    inline void printVec2(const glm::vec2 &vec, const std::string &name) {}
    inline void printVec3(const glm::vec3 &vec, const std::string &name) {}
    inline void printVec4(const glm::vec4 &vec, const std::string &name) {}
    inline void printQuat(const glm::quat &quat, const std::string &name) {}
    inline void printMat4(const glm::mat4 &mat, const std::string &name) {}
    inline void printSphere(const primitive::Sphere &sphere, const std::string &name) {}
    inline void printRay(const primitive::Ray &ray, const std::string &name) {}
    inline void printPlane(const primitive::Plane &plane, const std::string &name) {}
    inline void printAabb(const primitive::Aabb &aabb, const std::string &name) {}
    inline void printCacheContents() {}
    inline void printSceneTree() {}
#else
#define DEBUG(x)                 \
  do {                           \
    std::cerr << x << std::endl; \
  } while (0)
#define DEBUGONE(x)                             \
  do {                                          \
    std::cerr << #x << " = " << x << std::endl; \
  } while (0)

    inline void printVec2(const glm::vec2 &vec, const std::string &name) {
      DEBUG(name << " = (" << vec.x << ", " << vec.y << ")");
    }

    inline void printVec3(const glm::vec3 &vec, const std::string &name) {
      DEBUG(name << " = (" << vec.x << ", " << vec.y << ", " << vec.z << ")");
    }

    inline void printVec4(const glm::vec4 &vec, const std::string &name) {
      DEBUG(name << " = (" << vec.x << ", " << vec.y << ", " << vec.z << ", " << vec.w << ")");
    }

    inline void printQuat(const glm::quat &quat, const std::string &name) {
      DEBUG(name << " = (" << quat.x << ", " << quat.y << ", " << quat.z << ", " << quat.w << ")");
    }

    inline void printMat4(const glm::mat4 &mat, const std::string &name) {
      DEBUG(name << " =");
      DEBUG(mat[0][0] << ", " << mat[1][0] << ", " << mat[2][0] << ", " << mat[3][0]);
      DEBUG(mat[0][1] << ", " << mat[1][1] << ", " << mat[2][1] << ", " << mat[3][1]);
      DEBUG(mat[0][2] << ", " << mat[1][2] << ", " << mat[2][2] << ", " << mat[3][2]);
      DEBUG(mat[0][3] << ", " << mat[1][3] << ", " << mat[2][3] << ", " << mat[3][3]);
    }

    inline void printSphere(const primitive::Sphere &sphere, const std::string &name) {
      DEBUG(name << " = (" << sphere.mCenter.x << ", " << sphere.mCenter.y << ", " << sphere.mCenter.z << "; " << sphere.mRadius
                 << ")");
    }

    inline void printRay(const primitive::Ray &ray, const std::string &name) {
      DEBUG(name << " = (" << ray.mOrigin.x << ", " << ray.mOrigin.y << ", " << ray.mOrigin.z << "; " << ray.mDirection.x
                 << ", " << ray.mDirection.y << ", " << ray.mDirection.z << ")");
    }

    inline void printPlane(const primitive::Plane &plane, const std::string &name) {
      DEBUG(name << " = (" << plane.mNormal.x << ", " << plane.mNormal.y << ", " << plane.mNormal.z << "; "
                 << plane.mNegativeDistance << ")");
    }

    inline void printAabb(const primitive::Aabb &aabb, const std::string &name) {
      DEBUG(name << " = (" << aabb.mBounds[0].x << ", " << aabb.mBounds[0].y << ", " << aabb.mBounds[0].z << ") -> ("
                 << aabb.mBounds[1].x << ", " << aabb.mBounds[1].y << ", " << aabb.mBounds[1].z << ")");
    }

    inline void printCacheContents() {
      DEBUG("Cache contents: ");
      PhongMaterial::printCacheContents();
      PbrMaterial::printCacheContents();
      StaticMesh::printCacheContents();
      Texture2d::printCacheContents();
      DEBUG("");
    }

    inline void printNode(const Node *node, std::string indentation) {
      static std::string type;
      static const Camera *camera;
      static const LightNode *lightNode;
      static const Renderable *renderable;
      static const Transform *transform;
      static const Skeleton *skeleton;

      transform = NULL;

      if ((camera = dynamic_cast<const Camera *>(node)))
        type = "Camera";
      else if ((lightNode = dynamic_cast<const LightNode *>(node)))
        type = "LightNode";
      else if ((renderable = dynamic_cast<const Renderable *>(node)))
        type = "Renderable";
      else if ((transform = dynamic_cast<const Transform *>(node)))
        type = "Transform";
      else if ((skeleton = dynamic_cast<const Skeleton *>(node)))
        type = "Skeleton";

      DEBUG(indentation << type << " (" << node << ")");

      if (transform) {
        indentation.append("  ");
        for (const Node *child : transform->children())
          printNode(child, indentation);
      }
    }

    inline void printSceneTree() {
      DEBUG("Scene tree contents: ");
      printNode(Scene::instance()->root(), std::string());
    }
#endif

  }  // namespace debug
}  // namespace wren

#endif  // DEBUG_HPP
