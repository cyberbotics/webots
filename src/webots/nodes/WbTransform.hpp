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

#ifndef WB_TRANSFORM_HPP
#define WB_TRANSFORM_HPP

//
// Description: a node that defines a 3D coordinate system transformation
//

#include "WbPose.hpp"

class WbTransform : public WbPose {
  Q_OBJECT

public:
  // constructors and destructor
  WbTransform(const WbTransform &other);
  WbTransform(const WbNode &other);
  ~WbTransform();

  // reimplemented functions
  int nodeType() const override { return WB_NODE_TRANSFORM; }

private:
  WbTransform &operator=(const WbTransform &);  // non copyable
  WbNode *clone() const override { return new WbTransform(*this); }
  void init();

private slots:
};

#endif
