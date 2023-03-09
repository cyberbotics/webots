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

#ifndef WB_RGB_HPP
#define WB_RGB_HPP

//
// Description: RGB color object, to be used in WbSFColor and WbMFColor values
//

#include <QtCore/QStringList>

#include <WbPrecision.hpp>

#include <cassert>

class WbRgb {
public:
  WbRgb() : mRed(0.0), mGreen(0.0), mBlue(0.0) {}
  WbRgb(double r, double g, double b) : mRed(r), mGreen(g), mBlue(b) {}
  WbRgb(uint8_t r, uint8_t g, uint8_t b) : mRed(r / 255.0), mGreen(g / 255.0), mBlue(b / 255.0) {}
  WbRgb(const WbRgb &c) : mRed(c.mRed), mGreen(c.mGreen), mBlue(c.mBlue) {}
  explicit WbRgb(const QString &string) {
    const QStringList splittedText = string.split(' ');
    assert(splittedText.count() == 3);
    mRed = splittedText[0].toDouble();
    mGreen = splittedText[1].toDouble();
    mBlue = splittedText[2].toDouble();
  }
  ~WbRgb() {}
  void setRed(double r) { mRed = r; }
  void setGreen(double g) { mGreen = g; }
  void setBlue(double b) { mBlue = b; }
  void setValue(double r, double g, double b) {
    mRed = r;
    mGreen = g;
    mBlue = b;
  }
  void setByteValue(uint8_t r, uint8_t g, uint8_t b) {
    mRed = r / 255.0;
    mGreen = g / 255.0;
    mBlue = b / 255.0;
  }
  double red() const { return mRed; }
  double green() const { return mGreen; }
  double blue() const { return mBlue; }
  uint8_t redByte() const { return mRed * 255.0 + 0.5; }
  uint8_t greenByte() const { return mGreen * 255.0 + 0.5; }
  uint8_t blueByte() const { return mBlue * 255.0 + 0.5; }
  WbRgb &operator=(const WbRgb &c) {
    mRed = c.mRed;
    mGreen = c.mGreen;
    mBlue = c.mBlue;
    return *this;
  }
  bool operator==(const WbRgb &c) const { return mRed == c.mRed && mGreen == c.mGreen && mBlue == c.mBlue; }
  bool operator!=(const WbRgb &c) const { return mRed != c.mRed || mGreen != c.mGreen || mBlue != c.mBlue; }
  QString toString(WbPrecision::Level level = WbPrecision::Level::DOUBLE_MAX) const {
    return QString("%1 %2 %3")
      .arg(WbPrecision::doubleToString(mRed, level))
      .arg(WbPrecision::doubleToString(mGreen, level))
      .arg(WbPrecision::doubleToString(mBlue, level));
  }

  // clamp RGB values in range [0.0, 1.0]
  bool clampValuesIfNeeded() {
    const bool redReset = clampValue(mRed);
    const bool greenReset = clampValue(mGreen);
    const bool blueReset = clampValue(mBlue);
    return redReset || greenReset || blueReset;
  }

private:
  double mRed, mGreen, mBlue;

  static bool clampValue(double &value) {
    if (value < 0.0)
      value = 0.0;
    else if (value > 1.0)
      value = 1.0;
    else
      return false;
    return true;
  }
};

#endif
