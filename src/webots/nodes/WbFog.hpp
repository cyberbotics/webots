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

#ifndef WB_FOG_HPP
#define WB_FOG_HPP

#include "WbBaseNode.hpp"

#include <wren/scene.h>

// TODO: only the first Fog node should be taken into account

class WbFog : public WbBaseNode {
  Q_OBJECT

public:
  static int numberOfFogInstances() { return cFogList.count(); }
  static WbFog *fogInstance() { return cFogList.count() > 0 ? cFogList.first() : NULL; }

  // constructors and destructor
  explicit WbFog(WbTokenizer *tokenizer = NULL);
  WbFog(const WbFog &other);
  explicit WbFog(const WbNode &other);
  virtual ~WbFog();

  // reimplemented public functions
  int nodeType() const override { return WB_NODE_FOG; }
  void createWrenObjects() override;
  void preFinalize() override;
  void postFinalize() override;

  int mode() const { return mWrenFogType; }

  QStringList fieldsToSynchronizeWithX3D() const override;

signals:
  void modeChanged();

private:
  static QList<WbFog *> cFogList;

  WbFog &operator=(const WbFog &);  // non copyable
  // reimplemented functions
  WbNode *clone() const override { return new WbFog(*this); }

  // other functions
  void init();
  void applyChangesToWren();
  bool isFirstInstance() const { return cFogList.first() == this; }
  void updateAndConnectIfNeeded();

  // user accessible fields
  WbSFColor *mColor;
  WbSFString *mFogType;
  WbSFDouble *mVisibilityRange;

  // other variables
  WrSceneFogType mWrenFogType;

private slots:
  void updateColor();
  void updateFogType();
  void updateVisibilityRange();
};

#endif
