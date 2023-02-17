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

#ifndef WB_LED_HPP
#define WB_LED_HPP

#include "WbMFColor.hpp"
#include "WbSolidDevice.hpp"

class WbSFBool;
class WbRgb;
class WbLight;
class WbMaterial;
class WbPbrAppearance;
class WbGroup;

class QDataStream;

class WbLed : public WbSolidDevice {
  Q_OBJECT

public:
  // constructors and destructor
  explicit WbLed(WbTokenizer *tokenizer = NULL);
  WbLed(const WbLed &other);
  explicit WbLed(const WbNode &other);
  virtual ~WbLed();

  // reimplemented public functions
  int nodeType() const override { return WB_NODE_LED; }
  void preFinalize() override;
  void postFinalize() override;
  void reset(const QString &id) override;

  // field accessors
  int colorsCount() const { return mColor->size(); }
  const WbRgb &color(int index) const { return mColor->item(index); }
  int value() const { return mValue; }
  const QList<WbPbrAppearance *> &pbrAppearances() const { return mPbrAppearances; }
  bool isGradual() const;

  void setValue(int value);  // value=0: off; value>=1: on (select color[value-1])

  void powerOn(bool) override;
  void handleMessage(QDataStream &) override;

protected slots:
  void updateChildren() override;
  virtual void updateIfNeeded(WbField *);

private:
  // user accessible fields
  WbMFColor *mColor;
  WbSFBool *mGradual;

  // other fields
  int mValue;
  QList<WbMaterial *> mMaterials;
  QList<WbPbrAppearance *> mPbrAppearances;
  QList<WbLight *> mLights;

  WbLed &operator=(const WbLed &);  // non copyable
  WbNode *clone() const override { return new WbLed(*this); }
  void init();

  void findMaterialsAndLights(const WbGroup *const group);
  void clearMaterialsAndLights() {
    mMaterials.clear();
    mLights.clear();
    mPbrAppearances.clear();
  }
  bool isAnyMaterialOrLightFound();
  void setMaterialsAndLightsColor();
};

#endif
