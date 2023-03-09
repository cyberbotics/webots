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

#ifndef WB_FIELD_MODEL_HPP
#define WB_FIELD_MODEL_HPP

//
// Description: a class that defines a model for a node's field
//   The model is used in WbNodeModels.
//

#include <QtCore/QString>
#include <WbValue.hpp>
#include <WbVariant.hpp>

class WbTokenizer;
class WbToken;
class WbWriter;

class WbFieldModel {
public:
  // create from tokenizer
  WbFieldModel(WbTokenizer *tokenizer, const QString &worldPath);

  // field name
  const QString &name() const { return mName; }

  // VRML export
  bool isVrml() const { return mIsVrml; }
  void write(WbWriter &writer) const;

  bool isDeprecated() const { return mIsDeprecated; }

  // Hidden field flag
  bool isHiddenField() const { return mIsHiddenField; }
  bool isHiddenParameter() const { return mIsHiddenParameter; }

  bool isUnconnected() const { return mIsUnconnected; }

  // default value
  WbValue *defaultValue() const { return mDefaultValue; }

  // accepted values
  bool isValueAccepted(const WbValue *value, int *refusedIndex) const;
  bool hasRestrictedValues() const { return !mAcceptedValues.isEmpty(); }
  const QList<WbVariant> acceptedValues() const { return mAcceptedValues; }

  // field type
  WbFieldType type() const { return mDefaultValue->type(); }
  bool isMultiple() const;
  bool isSingle() const;

  // useful tokens for error reporting
  WbToken *nameToken() const { return mNameToken; }

  // template
  void setTemplateRegenerator(bool isRegenerator) { mIsTemplateRegenerator = isRegenerator; }
  bool isTemplateRegenerator() const { return mIsTemplateRegenerator; }

  // add/remove a reference to this field model from a field, a proto model or a node model instance
  // when the reference count reaches zero (in unref()) the field model is deleted
  void ref() const;
  void unref() const;

  // delete this field model
  // reference count has to be zero
  void destroy();

private:
  WbFieldModel(const WbFieldModel &);             // non constructor-copyable
  WbFieldModel &operator=(const WbFieldModel &);  // non copyable
  ~WbFieldModel();

  QString mName;
  bool mIsVrml;
  bool mIsHiddenField, mIsHiddenParameter;
  bool mIsTemplateRegenerator;
  bool mIsDeprecated;
  bool mIsUnconnected;
  WbValue *mDefaultValue;
  QList<WbVariant> mAcceptedValues;  // TODO: const WbVariant
  WbToken *mNameToken;

  mutable int mRefCount;

  static WbValue *createValueForVrmlType(const QString &type, WbTokenizer *tokenizer, const QString &worldPath);
  static QList<WbVariant> getAcceptedValues(const QString &type, WbTokenizer *tokenizer, const QString &worldPath);
};

#endif
