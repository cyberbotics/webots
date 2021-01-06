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

#ifndef WB_VISUAL_BOUNDING_SPHERE_HPP
#define WB_VISUAL_BOUNDING_SPHERE_HPP

//
// Description: Utility class to debug bounding spheres by graphically
//              displaying the sphere in the 3D scene.
//

#include "QtCore/QObject"

class WbBaseNode;

class WbVisualBoundingSphere : public QObject {
  Q_OBJECT

public:
  static WbVisualBoundingSphere *instance();

  static void enable(bool enabled, const WbBaseNode *node = NULL);

public slots:
  void show(const WbBaseNode *node);

private:
  static WbVisualBoundingSphere *cInstance;
  static void createSphere(/* const Ogre::String &meshName, const float r, const int nRings = 16, const int nSegments = 16 */);

  WbVisualBoundingSphere();
  ~WbVisualBoundingSphere();
  void clear();
};

#endif  // WB_VISUAL_BOUNDING_SPHERE_HPP
