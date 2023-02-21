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

#ifndef WB_PROTO_TEMPLATE_ENGINE_HPP
#define WB_PROTO_TEMPLATE_ENGINE_HPP

//
// Description:    template engine
// Responsability: manage file parsing using a template engine given a VRML context

#include <QtCore/QObject>
#include <QtCore/QString>

#include "WbTemplateEngine.hpp"

class WbField;
class WbVariant;

class WbProtoTemplateEngine : public WbTemplateEngine {
  Q_OBJECT

public:
  explicit WbProtoTemplateEngine(const QString &templateContent);
  virtual ~WbProtoTemplateEngine() {}

  bool generate(const QString &logHeaderName, const QVector<WbField *> &parameters, const QString &protoPath,
                const QString &worldPath, int id, const QString &templateLanguage);
  static QString convertFieldValueToJavaScriptStatement(const WbField *field);
  static const QString &coordinateSystem();
  static void setCoordinateSystem(const QString &coordinateSystem);
  static QString convertStatementFromJavaScriptToLua(QString &statement);

private:
  static QString convertFieldDefaultValueToJavaScriptStatement(const WbField *field);
  static QString convertVariantToJavaScriptStatement(const WbVariant &variant);
};

#endif
