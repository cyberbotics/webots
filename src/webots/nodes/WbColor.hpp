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

#ifndef WB_COLOR_HPP
#define WB_COLOR_HPP

#include "WbBaseNode.hpp"

class WbColor : public WbBaseNode {
  Q_OBJECT

public:
  // constructors and destructor
  explicit WbColor(WbTokenizer *tokenizer = NULL);
  WbColor(const WbColor &other);
  explicit WbColor(const WbNode &other);
  virtual ~WbColor();

  // reimplemented public functions
  int nodeType() const override { return WB_NODE_COLOR; }
  void preFinalize() override;
  void postFinalize() override;

  // field accessors
  const WbMFColor &color() const { return *mColor; }

  // helper, the size of receiving array should be equal to (or greater than) the number of mColor items
  void copyValuesToArray(double array[][3]) const;

  QStringList fieldsToSynchronizeWithX3D() const override;

signals:
  void changed();

protected slots:
  void updateColor();

private:
  // user accessible fields
  WbMFColor *mColor;

  WbColor &operator=(const WbColor &);  // non copyable
  WbNode *clone() const override { return new WbColor(*this); }
  void init();
};

#endif
