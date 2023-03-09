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

#ifndef WB_ANCHOR_PARAMETER_HPP
#define WB_ANCHOR_PARAMETER_HPP

#include "WbBaseNode.hpp"
#include "WbSFVector3.hpp"

class WbAnchorParameter : public WbBaseNode {
  Q_OBJECT

public:
  virtual ~WbAnchorParameter();

  const WbVector3 &anchor() const { return mAnchor->value(); }
  void preFinalize() override;
  void postFinalize() override;

signals:
  void anchorChanged();

protected:
  WbAnchorParameter(const QString &modelName, WbTokenizer *tokenizer);
  WbAnchorParameter(const WbAnchorParameter &other);
  explicit WbAnchorParameter(const WbNode &other);
  WbSFVector3 *mAnchor;

private:
  WbAnchorParameter &operator=(const WbAnchorParameter &);  // non copyable
  void init();
};

#endif
