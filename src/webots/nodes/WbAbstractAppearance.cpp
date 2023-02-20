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

#include "WbAbstractAppearance.hpp"

#include "WbSFNode.hpp"
#include "WbTextureTransform.hpp"
#include "WbVector2.hpp"

#include <assimp/material.h>

void WbAbstractAppearance::init() {
  mName = findSFString("name");
  mTextureTransform = findSFNode("textureTransform");
  mNameValue = mName->value();
}

WbAbstractAppearance::WbAbstractAppearance(const QString &modelName, WbTokenizer *tokenizer) :
  WbBaseNode(modelName, tokenizer) {
  init();
}

WbAbstractAppearance::WbAbstractAppearance(const WbAbstractAppearance &other) : WbBaseNode(other) {
  init();
}

WbAbstractAppearance::WbAbstractAppearance(const WbNode &other) : WbBaseNode(other) {
  init();
}

WbAbstractAppearance::WbAbstractAppearance(const QString &modelName, const aiMaterial *material) : WbBaseNode(modelName) {
  aiString nameString("PBRAppearance");
  material->Get(AI_MATKEY_NAME, nameString);
  mName = new WbSFString(QString(nameString.C_Str()));
  mNameValue = mName->value();

  mTextureTransform = new WbSFNode(NULL);
}

WbAbstractAppearance::~WbAbstractAppearance() {
  if (mIsShallowNode) {
    delete mName;
    delete mTextureTransform;
  }
}

void WbAbstractAppearance::preFinalize() {
  WbBaseNode::preFinalize();
  if (textureTransform())
    textureTransform()->preFinalize();

  updateTextureTransform();
}

void WbAbstractAppearance::postFinalize() {
  WbBaseNode::postFinalize();

  if (textureTransform())
    textureTransform()->postFinalize();

  connect(mName, &WbSFString::changed, this, &WbAbstractAppearance::updateName);
  connect(mTextureTransform, &WbSFNode::changed, this, &WbAbstractAppearance::updateTextureTransform);
}

void WbAbstractAppearance::reset(const QString &id) {
  WbBaseNode::reset(id);
  if (textureTransform())
    textureTransform()->reset(id);
}

void WbAbstractAppearance::createWrenObjects() {
  WbBaseNode::createWrenObjects();
  updateName();

  if (textureTransform())
    textureTransform()->createWrenObjects();
}

void WbAbstractAppearance::updateTextureTransform() {
  if (textureTransform())
    connect(textureTransform(), &WbTextureTransform::changed, this, &WbAbstractAppearance::updateTextureTransform,
            Qt::UniqueConnection);

  if (isPostFinalizedCalled())
    emit changed();
}

void WbAbstractAppearance::updateName() {
  if (isPostFinalizedCalled())
    emit nameChanged(mName->value(), mNameValue);
  mNameValue = mName->value();
}

WbTextureTransform *WbAbstractAppearance::textureTransform() const {
  return dynamic_cast<WbTextureTransform *>(mTextureTransform->value());
}

WbVector2 WbAbstractAppearance::transformUVCoordinate(const WbVector2 &uv) const {
  WbTextureTransform *tt = textureTransform();
  if (tt)
    return tt->transformUVCoordinate(uv);
  return uv;
}
