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

#ifndef WB_TEXTURE_COORDINATE_HPP
#define WB_TEXTURE_COORDINATE_HPP

#include "WbBaseNode.hpp"
#include "WbMFVector2.hpp"

class WbVector2;

class WbTextureCoordinate : public WbBaseNode {
  Q_OBJECT

public:
  // constructors and destructor
  explicit WbTextureCoordinate(WbTokenizer *tokenizer = NULL);
  WbTextureCoordinate(const WbTextureCoordinate &other);
  explicit WbTextureCoordinate(const WbNode &other);
  virtual ~WbTextureCoordinate();

  // reimplemented public functions
  int nodeType() const override { return WB_NODE_TEXTURE_COORDINATE; }

  // field accessors
  const WbMFVector2 &point() const { return *mPoint; }
  const WbVector2 &point(int index) const { return mPoint->item(index); }
  int pointSize() const { return mPoint->size(); }

  QStringList fieldsToSynchronizeWithX3D() const override;

private:
  // user accessible fields
  WbMFVector2 *mPoint;

  WbTextureCoordinate &operator=(const WbTextureCoordinate &);  // non copyable
  WbNode *clone() const override { return new WbTextureCoordinate(*this); }
  void init();
};

#endif
