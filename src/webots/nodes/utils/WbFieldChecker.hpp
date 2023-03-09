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
  static bool resetDoubleIfNegative(const WbBaseNode *node, WbSFDouble *value, double defaultValue);
  static bool resetDoubleIfNonPositive(const WbBaseNode *node, WbSFDouble *value, double defaultValue);
  static bool resetDoubleIfNotInRangeWithIncludedBounds(const WbBaseNode *node, WbSFDouble *value, double min, double max,
                                                        double defaultValue);
  static bool resetDoubleIfNotInRangeWithExcludedBounds(const WbBaseNode *node, WbSFDouble *value, double min, double max,
                                                        double defaultValue);
  static bool resetDoubleIfLess(const WbBaseNode *node, WbSFDouble *value, double threshold, double defaultValue);
  static bool resetDoubleIfGreater(const WbBaseNode *node, WbSFDouble *value, double threshold, double defaultValue);
  static bool resetDoubleIfNegativeAndNotDisabled(const WbBaseNode *node, WbSFDouble *value, double defaultValue,
                                                  double disableValue);
  static bool resetDoubleIfNonPositiveAndNotDisabled(const WbBaseNode *node, WbSFDouble *value, double defaultValue,
                                                     double disableValue);
  static bool resetDoubleIfNotInRangeWithIncludedBoundsAndNotDisabled(const WbBaseNode *node, WbSFDouble *value, double min,
                                                                      double max, double disableValue, double defaultValue);

  static bool clampDoubleToRangeWithIncludedBounds(const WbBaseNode *node, WbSFDouble *value, double min, double max);

  static bool resetIntIfNegative(const WbBaseNode *node, WbSFInt *value, int defaultValue);
  static bool resetIntIfNonPositive(const WbBaseNode *node, WbSFInt *value, int defaultValue);
  static bool resetIntIfLess(const WbBaseNode *node, WbSFInt *value, int threshold, int defaultValue);
  static bool resetIntIfNotInRangeWithIncludedBounds(const WbBaseNode *node, WbSFInt *value, int min, int max,
                                                     int defaultValue);
  static bool resetIntIfNonPositiveAndNotDisabled(const WbBaseNode *node, WbSFInt *value, int defaultValue, int disableValue);
  static bool resetIntIfNegativeAndNotDisabled(const WbBaseNode *node, WbSFInt *value, int defaultValue, int disableValue);

  static bool resetVector2IfNonPositive(const WbBaseNode *node, WbSFVector2 *value, const WbVector2 &defaultValue);

  static bool resetVector3IfNegative(const WbBaseNode *node, WbSFVector3 *value, const WbVector3 &defaultValue);
  static bool resetVector3IfNonPositive(const WbBaseNode *node, WbSFVector3 *value, const WbVector3 &defaultValue);

  static bool resetColorIfInvalid(const WbBaseNode *node, WbSFColor *value);
  static bool resetMultipleColorIfInvalid(const WbBaseNode *node, WbMFColor *value);

private:
  static const WbField *findField(const WbBaseNode *node, WbValue *value);

  static bool clampRgb(WbRgb &rgb);

  WbFieldChecker() {}
};

#endif
