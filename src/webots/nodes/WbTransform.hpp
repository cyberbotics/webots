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

#ifndef WB_TRANSFORM_HPP
#define WB_TRANSFORM_HPP

#include "WbPose.hpp"

class WbTransform : public WbPose {
  Q_OBJECT

public:
  // constructors and destructor
  explicit WbTransform(WbTokenizer *tokenizer = NULL);
  WbTransform(const WbTransform &other);
  explicit WbTransform(const WbNode &other);
  virtual ~WbTransform();

  // reimplemented functions
  int nodeType() const override { return WB_NODE_TRANSFORM; }

private:
  WbTransform &operator=(const WbTransform &);  // non copyable
  WbNode *clone() const override { return new WbTransform(*this); }
  void init();
};

#endif
