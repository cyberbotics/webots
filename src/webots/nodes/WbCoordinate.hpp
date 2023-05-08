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

#ifndef WB_COORDINATE_HPP
#define WB_COORDINATE_HPP

#include "WbBaseNode.hpp"
#include "WbMFVector3.hpp"

class WbVector3;

class WbCoordinate : public WbBaseNode {
  Q_OBJECT

public:
  // constructors and destructor
  explicit WbCoordinate(WbTokenizer *tokenizer = NULL);
  WbCoordinate(const WbCoordinate &other);
  explicit WbCoordinate(const WbNode &other);
  virtual ~WbCoordinate();

  // reimplemented public functions
  int nodeType() const override { return WB_NODE_COORDINATE; }

  // field accessors
  const WbMFVector3 &point() const { return *mPoint; }
  const WbVector3 &point(int index) const { return mPoint->item(index); }
  int pointSize() const { return mPoint->size(); }
  void rescale(const WbVector3 &v) { mPoint->rescale(v); }
  void rescaleAndTranslate(int coordinate, double scale, double translation) {
    mPoint->rescaleAndTranslate(coordinate, scale, translation);
  }
  void rescaleAndTranslate(const WbVector3 &s, const WbVector3 &t) { mPoint->rescaleAndTranslate(s, t); }
  void translate(const WbVector3 &v) { mPoint->translate(v); }
  QStringList fieldsToSynchronizeWithX3D() const override;

private:
  // user accessible fields
  WbMFVector3 *mPoint;

  WbCoordinate &operator=(const WbCoordinate &);  // non copyable
  WbNode *clone() const override { return new WbCoordinate(*this); }
  void init();
};

#endif
