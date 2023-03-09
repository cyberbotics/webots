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

#include "WbWrenAbstractManipulator.hpp"

#include "WbVector2.hpp"
#include "WbWrenRenderingContext.hpp"

#include <wren/node.h>
#include <wren/scene.h>

WrViewport *WbWrenAbstractManipulator::cViewport = NULL;

void WbWrenAbstractManipulator::setViewport(WrViewport *viewport) {
  cViewport = viewport;
}

WbWrenAbstractManipulator::WbWrenAbstractManipulator(int numberOfHandles) :
  mRootNode(NULL),
  mHandlesShader(NULL),
  mHandlesPickingShader(NULL),
  mIsVisible(false),
  mOriginScaleFactorNeeded(false),
  mScale(1.0),
  mNumberOfHandles(numberOfHandles),
  mIsActive(false) {
  mTransform = wr_transform_new();
  wr_node_set_visible(WR_NODE(mTransform), false);
}

WbWrenAbstractManipulator::~WbWrenAbstractManipulator() {
  wr_node_delete(WR_NODE(mTransform));
}

void WbWrenAbstractManipulator::attachTo(WrTransform *parent) {
  assert(parent);
  mRootNode = parent;
  wr_transform_attach_child(mRootNode, WR_NODE(mTransform));
}

void WbWrenAbstractManipulator::show() {
  mIsVisible = true;
  wr_node_set_visible(WR_NODE(mTransform), true);
  computeHandleScaleFromViewportSize();
}

void WbWrenAbstractManipulator::hide() {
  mIsVisible = false;
  wr_node_set_visible(WR_NODE(mTransform), false);
}

void WbWrenAbstractManipulator::showNormal() {
  computeHandleScaleFromViewportSize();
}

void WbWrenAbstractManipulator::updateHandleScale(const double *scale) {
  const float unscale[3] = {static_cast<float>(1.0f / scale[0]), static_cast<float>(1.0f / scale[1]),
                            static_cast<float>(1.0f / scale[2])};

  wr_transform_set_scale(mTransform, unscale);
}

void WbWrenAbstractManipulator::computeHandleScaleFromViewportSize() {
  const float sizeOnScreen = 100;
  float width = wr_viewport_get_width(cViewport), height = wr_viewport_get_height(cViewport),
        maxDimension = height > width ? height : width;
  mScale = 2 * sizeOnScreen / maxDimension;

  wr_shader_program_set_custom_uniform_value(mHandlesShader, "screenScale", WR_SHADER_PROGRAM_UNIFORM_TYPE_FLOAT,
                                             reinterpret_cast<const char *>(&mScale));
  wr_shader_program_set_custom_uniform_value(mHandlesPickingShader, "screenScale", WR_SHADER_PROGRAM_UNIFORM_TYPE_FLOAT,
                                             reinterpret_cast<const char *>(&mScale));
}
