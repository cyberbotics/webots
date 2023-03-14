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

#include "WbFieldModel.hpp"

#include "WbMFBool.hpp"
#include "WbMFColor.hpp"
#include "WbMFDouble.hpp"
#include "WbMFInt.hpp"
#include "WbMFNode.hpp"
#include "WbMFRotation.hpp"
#include "WbMFString.hpp"
#include "WbMFVector2.hpp"
#include "WbMFVector3.hpp"

#include "WbSFBool.hpp"
#include "WbSFColor.hpp"
#include "WbSFDouble.hpp"
#include "WbSFInt.hpp"
#include "WbSFNode.hpp"
#include "WbSFRotation.hpp"
#include "WbSFString.hpp"
#include "WbSFVector2.hpp"
#include "WbSFVector3.hpp"

#include "WbNode.hpp"
#include "WbToken.hpp"
#include "WbTokenizer.hpp"
#include "WbValue.hpp"
#include "WbWriter.hpp"

#include <cassert>

WbFieldModel::WbFieldModel(WbTokenizer *tokenizer, const QString &worldPath) {
  QString nw(tokenizer->nextWord());
  if (nw != "field" && nw != "vrmlField" && nw != "hiddenField" && nw != "hidden" && nw != "deprecatedField" &&
      nw != "unconnectedField") {
    tokenizer->reportError(QObject::tr("Expected field type but found '%2'").arg(nw), tokenizer->lastToken());
    throw 0;
  }

  mIsVrml = nw == "vrmlField";
  mIsDeprecated = nw == "deprecatedField";
  mIsHiddenField = mIsDeprecated || nw == "hiddenField";
  mIsHiddenParameter = nw == "hidden";
  mIsUnconnected = nw == "unconnectedField";
  mRefCount = 0;
  mIsTemplateRegenerator = false;

  QString typeName;
  if (mIsHiddenParameter) {
    nw = tokenizer->peekWord();

    if (nw.startsWith("rotation"))
      typeName = "SFRotation";
    else if (nw.startsWith("translation"))
      typeName = "SFVec3f";
    else if (nw.startsWith("position"))  // position or position2
      typeName = "SFFloat";
    else if (nw.startsWith("linearVelocity"))
      typeName = "SFVec3f";
    else if (nw.startsWith("angularVelocity"))
      typeName = "SFVec3f";
    else {
      tokenizer->reportError(QObject::tr("Expected hidden field identifier but found '%2'").arg(nw), tokenizer->lastToken());
      throw 0;
    }
  } else
    typeName = tokenizer->nextWord();

  if (tokenizer->nextWord() == "{") {
    QString singleTypeName = typeName;
    singleTypeName.replace("MF", "SF");
    mAcceptedValues = getAcceptedValues(singleTypeName, tokenizer, worldPath);
  } else
    tokenizer->ungetToken();

  // copy the token, indeed, the pointer reference can be deleted by the tokenizer
  mNameToken = new WbToken(*(tokenizer->nextToken()));
  mName = mNameToken->word();
  mDefaultValue = createValueForVrmlType(typeName, tokenizer, worldPath);
  if (mDefaultValue == NULL) {
    tokenizer->reportError(QObject::tr("Expected VRML97 type but found '%2'").arg(typeName), tokenizer->lastToken());
    throw 0;
  }

  if (hasRestrictedValues()) {
    int refusedIndex;
    bool defaultValueIsValid = true;
    while (!isValueAccepted(mDefaultValue, &refusedIndex)) {
      defaultValueIsValid = false;
      WbMultipleValue *multipleValue = dynamic_cast<WbMultipleValue *>(mDefaultValue);
      if (multipleValue)
        mAcceptedValues << multipleValue->variantValue(refusedIndex);
      else {
        WbSingleValue *singleValue = dynamic_cast<WbSingleValue *>(mDefaultValue);
        assert(singleValue);
        mAcceptedValues << singleValue->variantValue();
      }
    }
    if (!defaultValueIsValid)
      tokenizer->reportError(QObject::tr("The default value of field '%1' is not in the list of accepted values").arg(mName),
                             tokenizer->lastToken());
  }
}

WbFieldModel::~WbFieldModel() {
  delete mDefaultValue;
  delete mNameToken;
}

void WbFieldModel::destroy() {
  assert(mRefCount == 0);
  delete this;
}

void WbFieldModel::ref() const {
  mRefCount++;
}

void WbFieldModel::unref() const {
  mRefCount--;
  if (mRefCount == 0)
    delete this;
}

WbValue *WbFieldModel::createValueForVrmlType(const QString &type, WbTokenizer *tokenizer, const QString &worldPath) {
  if (type == "SFString")
    return new WbSFString(tokenizer, worldPath);
  else if (type == "SFInt32")
    return new WbSFInt(tokenizer, worldPath);
  else if (type == "SFFloat")
    return new WbSFDouble(tokenizer, worldPath);
  else if (type == "SFVec2f")
    return new WbSFVector2(tokenizer, worldPath);
  else if (type == "SFVec3f")
    return new WbSFVector3(tokenizer, worldPath);
  else if (type == "SFColor")
    return new WbSFColor(tokenizer, worldPath);
  else if (type == "SFNode")
    return new WbSFNode(tokenizer, worldPath);
  else if (type == "SFBool")
    return new WbSFBool(tokenizer, worldPath);
  else if (type == "SFRotation")
    return new WbSFRotation(tokenizer, worldPath);
  else if (type == "MFString")
    return new WbMFString(tokenizer, worldPath);
  else if (type == "MFInt32")
    return new WbMFInt(tokenizer, worldPath);
  else if (type == "MFFloat")
    return new WbMFDouble(tokenizer, worldPath);
  else if (type == "MFVec2f")
    return new WbMFVector2(tokenizer, worldPath);
  else if (type == "MFVec3f")
    return new WbMFVector3(tokenizer, worldPath);
  else if (type == "MFColor")
    return new WbMFColor(tokenizer, worldPath);
  else if (type == "MFNode")
    return new WbMFNode(tokenizer, worldPath);
  else if (type == "MFBool")
    return new WbMFBool(tokenizer, worldPath);
  else if (type == "MFRotation")
    return new WbMFRotation(tokenizer, worldPath);
  else
    return NULL;
}

QList<WbVariant> WbFieldModel::getAcceptedValues(const QString &type, WbTokenizer *tokenizer, const QString &worldPath) {
  QList<WbVariant> values;
  while (tokenizer->nextWord() != '}') {
    tokenizer->ungetToken();
    const WbSingleValue *singleValue =
      dynamic_cast<const WbSingleValue *>(WbFieldModel::createValueForVrmlType(type, tokenizer, worldPath));
    assert(singleValue);

    WbVariant variant(singleValue->variantValue());
    if (type == "SFNode" && variant.toNode()) {
      // explicit copy of the node to be persistent.
      WbNode *copy = variant.toNode()->cloneAndReferenceProtoInstance();
      variant.setNode(copy, true);
      QObject::connect(&variant, &QObject::destroyed, copy, &QObject::deleteLater);
    }

    values << variant;
    delete singleValue;
  }
  return values;
}

bool WbFieldModel::isValueAccepted(const WbValue *value, int *refusedIndex) const {
  *refusedIndex = -1;
  if (mAcceptedValues.isEmpty())
    return true;
  const WbMultipleValue *multipleValue = dynamic_cast<const WbMultipleValue *>(value);
  const WbSingleValue *singleValue = dynamic_cast<const WbSingleValue *>(value);
  if (multipleValue) {
    for (int i = 0; i < multipleValue->size(); ++i) {
      bool accepted = false;
      if (type() == WB_MF_NODE) {
        const WbMFNode *mfNode = static_cast<const WbMFNode *>(value);
        assert(mfNode);
        foreach (const WbVariant acceptedVariant, mAcceptedValues) {
          const WbNode *nodeAccepted = acceptedVariant.toNode();
          if (nodeAccepted && (mfNode->item(i)->nodeModelName() == nodeAccepted->modelName() ||
                               mfNode->item(i)->modelName() == nodeAccepted->modelName())) {
            accepted = true;
            break;
          }
        }
      } else {
        foreach (const WbVariant acceptedVariant, mAcceptedValues) {
          if (multipleValue->variantValue(i) == acceptedVariant) {
            accepted = true;
            break;
          }
        }
      }
      if (!accepted) {
        *refusedIndex = i;
        return false;
      }
    }
    return true;
  } else {
    assert(singleValue);
    foreach (const WbVariant acceptedVariant, mAcceptedValues) {
      if (type() == WB_SF_NODE) {
        const WbSFNode *sfNode = static_cast<const WbSFNode *>(value);
        assert(sfNode);
        if (!sfNode->value())
          return true;
        const WbNode *nodeAccepted = acceptedVariant.toNode();
        assert(nodeAccepted);
        if (sfNode->value()->nodeModelName() == nodeAccepted->modelName() ||
            sfNode->value()->modelName() == nodeAccepted->modelName())
          return true;
      } else if (singleValue->variantValue() == acceptedVariant)
        return true;
    }
    *refusedIndex = 0;
  }
  return false;
}

bool WbFieldModel::isMultiple() const {
  return dynamic_cast<WbMultipleValue *>(mDefaultValue);
}

bool WbFieldModel::isSingle() const {
  return dynamic_cast<WbSingleValue *>(mDefaultValue);
}

void WbFieldModel::write(WbWriter &writer) const {
  writer << "field " << mDefaultValue->vrmlTypeName() << " " << mName << " ";
  mDefaultValue->write(writer);
}
