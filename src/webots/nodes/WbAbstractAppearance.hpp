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

#ifndef WB_ABSTRACT_APPEARANCE_HPP
#define WB_ABSTRACT_APPEARANCE_HPP

#include "WbBaseNode.hpp"
#include "WbSFString.hpp"

class WbSFNode;
class WbTextureTransform;

struct WrMaterial;

struct aiMaterial;

class WbAbstractAppearance : public WbBaseNode {
  Q_OBJECT

public:
  virtual ~WbAbstractAppearance();

  void preFinalize() override;
  void postFinalize() override;
  void createWrenObjects() override;
  void reset(const QString &id) override;

  const QString &name() const { return mName->value(); }

  virtual WrMaterial *modifyWrenMaterial(WrMaterial *wrenMaterial) = 0;
  virtual WbTextureTransform *textureTransform() const;

  WbVector2 transformUVCoordinate(const WbVector2 &uv) const;

signals:
  void changed();
  void nameChanged(const QString &newName, const QString &prevName);

protected:
  WbAbstractAppearance(const QString &modelName, WbTokenizer *tokenizer = NULL);
  WbAbstractAppearance(const WbAbstractAppearance &other);
  WbAbstractAppearance(const WbNode &other);
  WbAbstractAppearance(const QString &modelName, const aiMaterial *material);

  WbSFNode *mTextureTransform;

private:
  WbAbstractAppearance &operator=(const WbAbstractAppearance &);  // non copyable
  void init();

  WbSFString *mName;
  QString mNameValue;

private slots:
  void updateName();
  void updateTextureTransform();
};

#endif
