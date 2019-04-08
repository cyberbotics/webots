// Copyright 1996-2019 Cyberbotics Ltd.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef WB_FIELD_CHECKER_HPP
#define WB_FIELD_CHECKER_HPP

class WbBaseNode;
class WbField;
class WbSFColor;
class WbSFDouble;
class WbSFInt;
class WbSFVector2;
class WbSFVector3;
class WbMFColor;
class WbRgb;
class WbValue;
class WbVector2;
class WbVector3;

#include <QtCore/QObject>

// NOTE: The pattern "return if changed" at the WbFieldChecker caller level is difficult to remove.
//       Indeed, I hoped solving that by blocking the signals of the WbValue when setting it to default
//       but this doesn't work in some cases (e.g. PROTOs)
//       Blocking the signals from value->node for a while is required to achieve,
//       and this is complex (and perhaps impossible):
//         http://stackoverflow.com/questions/15633086/qt-block-temporarily-signals-between-2-qobjects

class WbFieldChecker : public QObject {
  Q_OBJECT

public:
  static bool checkDoubleIsNonNegative(const WbBaseNode *node, WbSFDouble *value, double defaultValue);
  static bool checkDoubleIsNonPositive(const WbBaseNode *node, WbSFDouble *value, double defaultValue);
  static bool checkDoubleIsPositive(const WbBaseNode *node, WbSFDouble *value, double defaultValue);
  static bool checkDoubleInRangeWithIncludedBounds(const WbBaseNode *node, WbSFDouble *value, double min, double max,
                                                   double defaultValue);
  static bool checkDoubleInRangeWithExcludedBounds(const WbBaseNode *node, WbSFDouble *value, double min, double max,
                                                   double defaultValue);
  static bool checkDoubleIsGreaterOrEqual(const WbBaseNode *node, WbSFDouble *value, double threshold, double defaultValue);
  static bool checkDoubleIsGreater(const WbBaseNode *node, WbSFDouble *value, double threshold, double defaultValue);
  static bool checkDoubleIsLessOrEqual(const WbBaseNode *node, WbSFDouble *value, double threshold, double defaultValue);
  static bool checkDoubleIsLess(const WbBaseNode *node, WbSFDouble *value, double threshold, double defaultValue);
  static bool checkDoubleIsNonNegativeOrDisabled(const WbBaseNode *node, WbSFDouble *value, double defaultValue,
                                                 double disableValue);
  static bool checkDoubleIsPositiveOrDisabled(const WbBaseNode *node, WbSFDouble *value, double defaultValue,
                                              double disableValue);
  static bool checkDoubleIsInRangeWithIncludedBoundsOrDisabled(const WbBaseNode *node, WbSFDouble *value, double min,
                                                               double max, double disableValue, double defaultValue);

  static bool checkAndClampDoubleInRangeWithIncludedBounds(const WbBaseNode *node, WbSFDouble *value, double min, double max);

  static bool checkIntIsNonNegative(const WbBaseNode *node, WbSFInt *value, int defaultValue);
  static bool checkIntIsPositive(const WbBaseNode *node, WbSFInt *value, int defaultValue);
  static bool checkIntIsGreaterOrEqual(const WbBaseNode *node, WbSFInt *value, int threshold, int defaultValue);
  static bool checkIntInRangeWithIncludedBounds(const WbBaseNode *node, WbSFInt *value, int min, int max, int defaultValue);
  static bool checkIntIsPositiveOrDisabled(const WbBaseNode *node, WbSFInt *value, int defaultValue, int disableValue);
  static bool checkIntIsNonNegativeOrDisabled(const WbBaseNode *node, WbSFInt *value, int defaultValue, int disableValue);

  static bool checkVector2IsPositive(const WbBaseNode *node, WbSFVector2 *value, const WbVector2 &defaultValue);

  static bool checkVector3IsNonNegative(const WbBaseNode *node, WbSFVector3 *value, const WbVector3 &defaultValue);
  static bool checkVector3IsPositive(const WbBaseNode *node, WbSFVector3 *value, const WbVector3 &defaultValue);

  static bool checkColorIsValid(const WbBaseNode *node, WbSFColor *value);
  static bool checkMultipleColorIsValid(const WbBaseNode *node, WbMFColor *value);

private:
  static const WbField *findField(const WbBaseNode *node, WbValue *value);

  static bool checkRgbIsValid(WbRgb &rgb);

  WbFieldChecker() {}
};

#endif
