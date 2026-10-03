// Copyright 1996-2024 Cyberbotics Ltd.
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

#include "Camera.hpp"
#include "Transform.hpp"

#include <cmath>
#include <functional>
#include <iostream>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace {
  typedef std::unique_ptr<wren::Camera, void (*)(wren::Node *)> CameraPtr;
  typedef std::unique_ptr<wren::Transform, void (*)(wren::Node *)> TransformPtr;

  CameraPtr newCamera(bool initializeFrustum = true) {
    CameraPtr camera(wren::Camera::createCamera(), wren::Node::deleteNode);
    camera->setOrientation(glm::quat(1.0f, 0.0f, 0.0f, 0.0f));
    if (initializeFrustum)
      camera->frustum();
    return camera;
  }

  bool visible(wren::Camera &camera, const glm::vec3 &point) {
    return camera.frustum().isInside(wren::primitive::Sphere(point, 0.1f));
  }

  int total = 0;
  int failures = 0;

  void check(const std::string &name, bool result) {
    ++total;
    failures += !result;
    std::cout << (result ? "PASS " : "FAIL ") << name << '\n';
  }
}  // namespace

int main() {
  const glm::vec3 oldPoint(0.0f, 0.0f, -10.0f);
  const glm::vec3 translatedPoint(100.0f, 0.0f, -10.0f);
  const glm::vec3 rotatedPoint(0.0f, 0.0f, 10.0f);
  const std::vector<std::pair<std::string, std::function<void(wren::Camera &)>>> readers = {
    {"view", [](wren::Camera &camera) { camera.view(); }},
    {"right", [](wren::Camera &camera) { camera.right(); }},
    {"up", [](wren::Camera &camera) { camera.up(); }},
    {"forward", [](wren::Camera &camera) { camera.forward(); }}};

  // Neither matrix getter should consume the first frustum update.
  {
    auto camera = newCamera(false);
    camera->view();
    camera->projection();
    // Avoid reading uninitialized planes if the frustum was never built.
    check("initial matrix reads before frustum", camera->frustum().corners().size() == 8 && visible(*camera, oldPoint));
  }
  {
    auto camera = newCamera(false);
    camera->projection();
    camera->view();
    check("initial projection then view", camera->frustum().corners().size() == 8 && visible(*camera, oldPoint));
  }
  {
    auto camera = newCamera();
    camera->setPosition(glm::vec3(100.0f, 0.0f, 0.0f));
    check("translation without getter", !visible(*camera, oldPoint) && visible(*camera, translatedPoint));
  }
  for (const auto &reader : readers) {
    auto camera = newCamera();
    const bool initiallyVisible = visible(*camera, oldPoint) && !visible(*camera, translatedPoint);
    camera->setPosition(glm::vec3(100.0f, 0.0f, 0.0f));
    reader.second(*camera);
    check("translation after " + reader.first,
          initiallyVisible && !visible(*camera, oldPoint) && visible(*camera, translatedPoint));
  }
  for (const auto &reader : readers) {
    auto camera = newCamera();
    camera->setOrientation(glm::angleAxis(glm::pi<float>(), wren::gVec3UnitY));
    reader.second(*camera);
    check("rotation after " + reader.first, !visible(*camera, oldPoint) && visible(*camera, rotatedPoint));
  }

  const std::vector<std::pair<std::string, std::function<void(wren::Camera &)>>> rotations = {
    {"quaternion rotation",
     [](wren::Camera &camera) { camera.applyRotation(glm::angleAxis(glm::pi<float>(), wren::gVec3UnitY)); }},
    {"axis rotation", [](wren::Camera &camera) { camera.applyRotation(glm::pi<float>(), wren::gVec3UnitY); }},
    {"direction", [](wren::Camera &camera) { camera.setDirection(wren::gVec3UnitZ); }},
    {"pitch", [](wren::Camera &camera) { camera.applyPitch(glm::pi<float>()); }},
    {"roll", [](wren::Camera &camera) { camera.applyRoll(glm::pi<float>()); }}};
  for (const auto &rotation : rotations) {
    auto camera = newCamera();
    rotation.second(*camera);
    camera->forward();
    check(rotation.first + " after forward", !visible(*camera, oldPoint) && visible(*camera, rotatedPoint));
  }
  {
    auto camera = newCamera();
    camera->setDirection(wren::gVec3UnitX);
    const bool initiallyVisible = visible(*camera, glm::vec3(10.0f, 0.0f, 0.0f));
    camera->applyYaw(glm::pi<float>());
    camera->right();
    check("yaw after right", initiallyVisible && !visible(*camera, glm::vec3(10.0f, 0.0f, 0.0f)) &&
                               visible(*camera, glm::vec3(-10.0f, 0.0f, 0.0f)));
  }
  {
    auto camera = newCamera();
    camera->applyTranslation(glm::vec3(100.0f, 0.0f, 0.0f));
    camera->view();
    check("incremental translation after view", !visible(*camera, oldPoint) && visible(*camera, translatedPoint));
  }

  // Projection checks use points well away from the planes, with an initial visibility witness.
  for (bool readProjection : {false, true}) {
    auto camera = newCamera();
    const glm::vec3 point(0.0f, 0.0f, -50.0f);
    const bool initiallyVisible = visible(*camera, point);
    camera->setFar(20.0f);
    if (readProjection)
      camera->projection();
    check(readProjection ? "far after projection" : "far without getter", initiallyVisible && !visible(*camera, point));
  }
  {
    auto camera = newCamera();
    const glm::vec3 point(0.0f, 0.0f, -1.0f);
    const bool initiallyVisible = visible(*camera, point);
    camera->setNear(5.0f);
    camera->projection();
    check("near after projection", initiallyVisible && !visible(*camera, point));
  }
  for (bool changeFovy : {false, true}) {
    auto camera = newCamera();
    const glm::vec3 point(6.0f, 0.0f, -10.0f);
    const bool initiallyOutside = !visible(*camera, point);
    if (changeFovy)
      camera->setFovy(glm::half_pi<float>());
    else
      camera->setAspectRatio(3.0f);
    camera->projection();
    check(changeFovy ? "fovy after projection" : "aspect after projection", initiallyOutside && visible(*camera, point));
  }
  {
    auto camera = newCamera();
    const glm::vec3 point(0.0f, 0.0f, -500.0f);
    const bool initiallyOutside = !visible(*camera, point);
    camera->setFar(0.0f);
    camera->projection();
    check("default far distance after projection", initiallyOutside && visible(*camera, point));
  }
  for (bool projectionFirst : {false, true}) {
    auto camera = newCamera();
    camera->setPosition(glm::vec3(100.0f, 0.0f, 0.0f));
    camera->setFar(20.0f);
    if (projectionFirst) {
      camera->projection();
      camera->view();
    } else {
      camera->view();
      camera->projection();
    }
    check(
      projectionFirst ? "projection then view" : "view then projection",
      visible(*camera, translatedPoint) && !visible(*camera, oldPoint) && !visible(*camera, glm::vec3(100.0f, 0.0f, -50.0f)));
  }
  {
    auto camera = newCamera();
    camera->setPosition(glm::vec3(100.0f, 0.0f, 0.0f));
    camera->view();
    camera->setFar(20.0f);
    check("remaining dirty projection control", visible(*camera, translatedPoint) && !visible(*camera, oldPoint));
  }
  {
    auto camera = newCamera();
    camera->setFar(20.0f);
    camera->projection();
    camera->setPosition(glm::vec3(100.0f, 0.0f, 0.0f));
    check("remaining dirty view control", visible(*camera, translatedPoint) && !visible(*camera, oldPoint));
  }
  {
    TransformPtr parent(wren::Transform::createTransform(), wren::Node::deleteNode);
    auto camera = newCamera();
    parent->attachChild(camera.get());
    parent->setPosition(glm::vec3(100.0f, 0.0f, 0.0f));
    // Follow scene traversal before reading the attached camera.
    parent->updateFromParent();
    camera->view();
    check("parent translation after view", !visible(*camera, oldPoint) && visible(*camera, translatedPoint));
    parent->setOrientation(glm::angleAxis(glm::pi<float>(), wren::gVec3UnitY));
    parent->updateFromParent();
    camera->forward();
    check("parent rotation after forward",
          !visible(*camera, translatedPoint) && visible(*camera, glm::vec3(100.0f, 0.0f, 10.0f)));
  }
  {
    auto camera = newCamera();
    // Projection mode changes must not consume the pending frustum update.
    camera->setProjectionMode(WR_CAMERA_PROJECTION_MODE_ORTHOGRAPHIC);
    camera->projection();
    camera->setProjectionMode(WR_CAMERA_PROJECTION_MODE_PERSPECTIVE);
    camera->setFar(20.0f);
    camera->projection();
    check("return to perspective after projection",
          visible(*camera, oldPoint) && !visible(*camera, glm::vec3(0.0f, 0.0f, -50.0f)));
  }
  {
    auto direct = newCamera();
    auto withGetter = newCamera();
    for (wren::Camera *camera : {direct.get(), withGetter.get()}) {
      camera->setProjectionMode(WR_CAMERA_PROJECTION_MODE_ORTHOGRAPHIC);
      camera->setWindow(2.0f, 2.0f);
      camera->setHeight(4.0f);
      camera->frustum();
      camera->setPosition(glm::vec3(100.0f, 0.0f, 0.0f));
    }
    withGetter->view();
    // Check getter order against direct updating, independently of the projection's geometry.
    check("orthographic translation getter order", direct->frustum().corners() == withGetter->frustum().corners());
  }
  {
    auto camera = newCamera();
    camera->setFlipY(true);
    camera->setPosition(glm::vec3(100.0f, 0.0f, 0.0f));
    camera->view();
    check("flipped camera translation after view", !visible(*camera, oldPoint) && visible(*camera, translatedPoint));
  }
  {
    auto camera = newCamera();
    camera->setPosition(glm::vec3(100.0f, 0.0f, 0.0f));
    camera->view();
    camera->setPosition(glm::vec3(200.0f, 0.0f, 0.0f));
    camera->right();
    check("multiple view mutations before query", !visible(*camera, oldPoint) && !visible(*camera, translatedPoint) &&
                                                    visible(*camera, glm::vec3(200.0f, 0.0f, -10.0f)));
  }
  {
    auto camera = newCamera();
    camera->setPosition(glm::vec3(100.0f, 0.0f, 0.0f));
    camera->view();
    const bool firstUpdate = !visible(*camera, oldPoint) && visible(*camera, translatedPoint);
    camera->setPosition(glm::vec3(0.0f));
    camera->forward();
    check("view mutation after frustum refresh",
          firstUpdate && visible(*camera, oldPoint) && !visible(*camera, translatedPoint));
  }
  {
    auto camera = newCamera();
    camera->setFar(20.0f);
    camera->projection();
    camera->setFar(80.0f);
    camera->projection();
    camera->setNear(5.0f);
    camera->projection();
    camera->setNear(4.0f);
    camera->projection();
    check("multiple projection mutations before query",
          visible(*camera, glm::vec3(0.0f, 0.0f, -50.0f)) && !visible(*camera, glm::vec3(0.0f, 0.0f, -1.0f)));
  }
  {
    auto camera = newCamera();
    const glm::vec3 point(0.0f, 0.0f, -50.0f);
    camera->setFar(20.0f);
    camera->projection();
    const bool firstUpdate = !visible(*camera, point);
    camera->setFar(80.0f);
    camera->projection();
    check("projection mutation after frustum refresh", firstUpdate && visible(*camera, point));
  }
  {
    auto camera = newCamera();
    const auto corners = camera->frustum().corners();
    const auto bounds = camera->frustum().aabb();
    const auto plane = camera->frustum().plane(wren::Frustum::FRUSTUM_PLANE_RIGHT);
    const glm::vec3 offset(100.0f, 0.0f, 0.0f);
    camera->setPosition(offset);
    camera->view();
    const auto &frustum = camera->frustum();
    bool moved = frustum.corners().size() == corners.size();
    // Corners are reconstructed through a float inverse, so allow 1 cm at this 100 m offset.
    for (size_t i = 0; moved && i < corners.size(); ++i)
      moved = glm::length(frustum.corners()[i] - corners[i] - offset) < 0.01f;
    for (int i = 0; i < 2; ++i)
      moved = moved && glm::length(frustum.aabb().mBounds[i] - bounds.mBounds[i] - offset) < 0.01f;
    const auto &newPlane = frustum.plane(wren::Frustum::FRUSTUM_PLANE_RIGHT);
    moved = moved && glm::length(newPlane.mNormal - plane.mNormal) < 0.001f &&
            std::abs(newPlane.mNegativeDistance - plane.mNegativeDistance + glm::dot(plane.mNormal, offset)) < 0.001f;
    check("frustum corners bounds and plane after view", moved);
  }
  {
    auto camera = newCamera();
    camera->setPosition(glm::vec3(100.0f, 0.0f, 0.0f));
    camera->view();
    camera->setFar(20.0f);
    camera->projection();
    camera->update();
    const glm::vec3 radius(0.1f);
    // Renderer visibility predicates use the cache after the camera's update phase.
    check("explicit update refreshes cached visibility",
          !camera->isBoundingSphereVisible(wren::primitive::Sphere(oldPoint, 0.1f)) &&
            camera->isBoundingSphereVisible(wren::primitive::Sphere(translatedPoint, 0.1f)) &&
            !camera->isAabbVisible(wren::primitive::Aabb(oldPoint - radius, oldPoint + radius)) &&
            camera->isAabbVisible(wren::primitive::Aabb(translatedPoint - radius, translatedPoint + radius)));
  }
  {
    auto camera = newCamera();
    const auto corners = camera->frustum().corners();
    bool unchanged = true;
    for (int i = 0; i < 10; ++i) {
      camera->view();
      camera->projection();
      camera->forward();
      unchanged = unchanged && corners == camera->frustum().corners() && visible(*camera, oldPoint);
    }
    check("unchanged camera repeated reads", unchanged);
  }
  std::cout << total - failures << '/' << total << " checks passed\n";
  return failures ? 1 : 0;
}
