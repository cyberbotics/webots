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

#ifndef WB_ZOOM_HPP
#define WB_ZOOM_HPP

#include "WbBaseNode.hpp"
#include "WbSFDouble.hpp"

class WbZoom : public WbBaseNode {
  Q_OBJECT

public:
  explicit WbZoom(WbTokenizer *tokenizer = NULL);
  WbZoom(const WbZoom &other);
  explicit WbZoom(const WbNode &other);
  virtual ~WbZoom();

  // reimplemented public functions
  int nodeType() const override { return WB_NODE_ZOOM; }
  void preFinalize() override;
  void postFinalize() override;

  // getters
  double minFieldOfView() const { return mMinFieldOfView->value(); }
  double maxFieldOfView() const { return mMaxFieldOfView->value(); }

private:
  WbZoom &operator=(const WbZoom &);  // non copyable
  WbNode *clone() const override { return new WbZoom(*this); }

  void init();

  WbSFDouble *mMinFieldOfView;
  WbSFDouble *mMaxFieldOfView;

private slots:
  void updateMinFieldOfView();
  void updateMaxFieldOfView();
};

#endif
