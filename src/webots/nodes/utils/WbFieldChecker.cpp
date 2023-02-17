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

#include "WbFieldChecker.hpp"

#include "WbBaseNode.hpp"
#include "WbField.hpp"
#include "WbMFColor.hpp"
#include "WbRgb.hpp"
#include "WbSFColor.hpp"
#include "WbSFDouble.hpp"
#include "WbSFInt.hpp"
#include "WbSFVector2.hpp"
#include "WbSFVector3.hpp"
#include "WbValue.hpp"
#include "WbVector2.hpp"
#include "WbVector3.hpp"

/*
In English:

- "x is positive" means "x > 0"
- "x is negative" means "x < 0"

Hence:

- "x is non-negative" means "x >= 0"
- "x is non-positive" means "x <= 0"
*/

bool WbFieldChecker::resetDoubleIfNegative(const WbBaseNode *node, WbSFDouble *value, double defaultValue) {
  if (value->value() < 0) {
    const WbField *field = findField(node, value);
    node->parsingWarn(tr("Invalid '%1' changed to %2. The value should be non-negative.").arg(field->name()).arg(defaultValue));
    value->setValue(defaultValue);
    return true;
  }
  return false;
}

bool WbFieldChecker::resetDoubleIfNonPositive(const WbBaseNode *node, WbSFDouble *value, double defaultValue) {
  if (value->value() <= 0) {
    const WbField *field = findField(node, value);
    node->parsingWarn(tr("Invalid '%1' changed to %2. The value should be positive.").arg(field->name()).arg(defaultValue));
    value->setValue(defaultValue);
    return true;
  }
  return false;
}

bool WbFieldChecker::resetDoubleIfNegativeAndNotDisabled(const WbBaseNode *node, WbSFDouble *value, double defaultValue,
                                                         double disableValue) {
  if (value->value() < 0 && (value->value() != disableValue)) {
    const WbField *field = findField(node, value);
    node->parsingWarn(tr("Invalid '%1' changed to %2. The value should be either %3 or non-negative.")
                        .arg(field->name())
                        .arg(defaultValue)
                        .arg(disableValue));
    value->setValue(defaultValue);
    return true;
  }
  return false;
}

bool WbFieldChecker::resetDoubleIfNonPositiveAndNotDisabled(const WbBaseNode *node, WbSFDouble *value, double defaultValue,
                                                            double disableValue) {
  if (value->value() <= 0 && (value->value() != disableValue)) {
    const WbField *field = findField(node, value);
    node->parsingWarn(tr("Invalid '%1' changed to %2. The value should be either %3 or positive.")
                        .arg(field->name())
                        .arg(defaultValue)
                        .arg(disableValue));
    value->setValue(defaultValue);
    return true;
  }
  return false;
}

bool WbFieldChecker::resetDoubleIfNotInRangeWithIncludedBoundsAndNotDisabled(const WbBaseNode *node, WbSFDouble *value,
                                                                             double min, double max, double disableValue,
                                                                             double defaultValue) {
  if (value->value() != disableValue && (value->value() < min || value->value() > max)) {
    const WbField *field = findField(node, value);
    node->parsingWarn(tr("Invalid '%1' changed to %2. The value should be in either %3 or in range [%4, %5].")
                        .arg(field->name())
                        .arg(defaultValue)
                        .arg(disableValue)
                        .arg(min)
                        .arg(max));
    value->setValue(defaultValue);
    return true;
  }
  return false;
}

bool WbFieldChecker::resetDoubleIfNotInRangeWithIncludedBounds(const WbBaseNode *node, WbSFDouble *value, double min,
                                                               double max, double defaultValue) {
  if (value->value() < min || value->value() > max) {
    const WbField *field = findField(node, value);
    node->parsingWarn(tr("Invalid '%1' changed to %2. The value should be in range [%3, %4].")
                        .arg(field->name())
                        .arg(defaultValue)
                        .arg(min)
                        .arg(max));
    value->setValue(defaultValue);
    return true;
  }
  return false;
}

bool WbFieldChecker::clampDoubleToRangeWithIncludedBounds(const WbBaseNode *node, WbSFDouble *value, double min, double max) {
  double defaultValue = value->value();
  if (value->value() < min)
    defaultValue = min;
  else if (value->value() > max)
    defaultValue = max;

  if (defaultValue != value->value()) {
    const WbField *field = findField(node, value);
    node->parsingWarn(tr("Invalid '%1' changed to %2. The value should be in range [%3, %4].")
                        .arg(field->name())
                        .arg(defaultValue)
                        .arg(min)
                        .arg(max));
    value->setValue(defaultValue);
    return true;
  }
  return false;
}

bool WbFieldChecker::resetDoubleIfNotInRangeWithExcludedBounds(const WbBaseNode *node, WbSFDouble *value, double min,
                                                               double max, double defaultValue) {
  if (value->value() <= min || value->value() >= max) {
    const WbField *field = findField(node, value);
    node->parsingWarn(tr("Invalid '%1' changed to %2. The value should be in range ]%3, %4[.")
                        .arg(field->name())
                        .arg(defaultValue)
                        .arg(min)
                        .arg(max));
    value->setValue(defaultValue);
    return true;
  }
  return false;
}

bool WbFieldChecker::resetDoubleIfLess(const WbBaseNode *node, WbSFDouble *value, double threshold, double defaultValue) {
  if (value->value() < threshold) {
    const WbField *field = findField(node, value);
    node->parsingWarn(
      tr("Invalid '%1' changed to %2. The value should be %3 or greater.").arg(field->name()).arg(defaultValue).arg(threshold));
    value->setValue(defaultValue);
    return true;
  }
  return false;
}

bool WbFieldChecker::resetDoubleIfGreater(const WbBaseNode *node, WbSFDouble *value, double threshold, double defaultValue) {
  if (value->value() > threshold) {
    const WbField *field = findField(node, value);
    node->parsingWarn(
      tr("Invalid '%1' changed to %2. The value should be %3 or less.").arg(field->name()).arg(defaultValue).arg(threshold));
    value->setValue(defaultValue);
    return true;
  }
  return false;
}

bool WbFieldChecker::resetIntIfNegative(const WbBaseNode *node, WbSFInt *value, int defaultValue) {
  if (value->value() < 0) {
    const WbField *field = findField(node, value);
    node->parsingWarn(tr("Invalid '%1' changed to %2. The value should be non-negative.").arg(field->name()).arg(defaultValue));
    value->setValue(defaultValue);
    return true;
  }
  return false;
}

bool WbFieldChecker::resetIntIfNonPositive(const WbBaseNode *node, WbSFInt *value, int defaultValue) {
  if (value->value() <= 0) {
    const WbField *field = findField(node, value);
    node->parsingWarn(tr("Invalid '%1' changed to %2. The value should be positive.").arg(field->name()).arg(defaultValue));
    value->setValue(defaultValue);
    return true;
  }
  return false;
}

bool WbFieldChecker::resetIntIfLess(const WbBaseNode *node, WbSFInt *value, int threshold, int defaultValue) {
  if (value->value() < threshold) {
    const WbField *field = findField(node, value);
    node->parsingWarn(
      tr("Invalid '%1' changed to %2. The value should be %3 or greater.").arg(field->name()).arg(defaultValue).arg(threshold));
    value->setValue(defaultValue);
    return true;
  }
  return false;
}

bool WbFieldChecker::resetIntIfNotInRangeWithIncludedBounds(const WbBaseNode *node, WbSFInt *value, int min, int max,
                                                            int defaultValue) {
  if (value->value() < min || value->value() > max) {
    const WbField *field = findField(node, value);
    node->parsingWarn(tr("Invalid '%1' changed to %2. The value should be in range [%3, %4].")
                        .arg(field->name())
                        .arg(defaultValue)
                        .arg(min)
                        .arg(max));
    value->setValue(defaultValue);
    return true;
  }
  return false;
}

bool WbFieldChecker::resetIntIfNonPositiveAndNotDisabled(const WbBaseNode *node, WbSFInt *value, int defaultValue,
                                                         int disableValue) {
  if (value->value() <= 0 && value->value() != disableValue) {
    const WbField *field = findField(node, value);
    node->parsingWarn(tr("Invalid '%1' changed to %2. The value should be either %3 or positive.")
                        .arg(field->name())
                        .arg(defaultValue)
                        .arg(disableValue));
    value->setValue(defaultValue);
    return true;
  }
  return false;
}

bool WbFieldChecker::resetIntIfNegativeAndNotDisabled(const WbBaseNode *node, WbSFInt *value, int defaultValue,
                                                      int disableValue) {
  if (value->value() < 0 && value->value() != disableValue) {
    const WbField *field = findField(node, value);
    node->parsingWarn(tr("Invalid '%1' changed to %2. The value should be either %3 or non-negative.")
                        .arg(field->name())
                        .arg(defaultValue)
                        .arg(disableValue));
    value->setValue(defaultValue);
    return true;
  }
  return false;
}

bool WbFieldChecker::resetVector2IfNonPositive(const WbBaseNode *node, WbSFVector2 *value, const WbVector2 &defaultValue) {
  if (value->x() <= 0 || value->y() <= 0) {
    const WbField *field = findField(node, value);
    node->parsingWarn(tr("Invalid '%1' changed to %2. The value should be positive.")
                        .arg(field->name())
                        .arg(defaultValue.toString(WbPrecision::GUI_MEDIUM)));
    value->setValue(defaultValue);
    return true;
  }
  return false;
}

bool WbFieldChecker::resetVector3IfNegative(const WbBaseNode *node, WbSFVector3 *value, const WbVector3 &defaultValue) {
  if (value->x() < 0 || value->y() < 0 || value->z() < 0) {
    const WbField *field = findField(node, value);
    node->parsingWarn(tr("Invalid '%1' changed to %2. The value should be non-negative.")
                        .arg(field->name())
                        .arg(defaultValue.toString(WbPrecision::GUI_MEDIUM)));
    value->setValue(defaultValue);
    return true;
  }
  return false;
}

bool WbFieldChecker::resetVector3IfNonPositive(const WbBaseNode *node, WbSFVector3 *value, const WbVector3 &defaultValue) {
  if (value->x() <= 0 || value->y() <= 0 || value->z() <= 0) {
    const WbField *field = findField(node, value);
    node->parsingWarn(tr("Invalid '%1' changed to %2. The value should be positive.")
                        .arg(field->name())
                        .arg(defaultValue.toString(WbPrecision::GUI_MEDIUM)));
    value->setValue(defaultValue);
    return true;
  }
  return false;
}

bool WbFieldChecker::resetColorIfInvalid(const WbBaseNode *node, WbSFColor *value) {
  WbRgb rgb = value->value();
  if (rgb.clampValuesIfNeeded()) {
    const WbField *field = findField(node, value);
    node->parsingWarn(tr("Invalid '%1' changed to %2.").arg(field->name()).arg(rgb.toString(WbPrecision::GUI_MEDIUM)));
    value->setValue(rgb);
    return true;
  }
  return false;
}

bool WbFieldChecker::resetMultipleColorIfInvalid(const WbBaseNode *node, WbMFColor *value) {
  bool changed = false;
  int size = value->size();
  const WbField *field = findField(node, value);
  for (int i = 0; i < size; i++) {
    WbRgb rgb = value->item(i);
    if (rgb.clampValuesIfNeeded()) {
      node->parsingWarn(
        tr("Invalid item %1 of '%2' changed to %3.").arg(i).arg(field->name()).arg(rgb.toString(WbPrecision::GUI_MEDIUM)));
      value->setItem(i, rgb);
      changed = true;
    }
  }
  return changed;
}

const WbField *WbFieldChecker::findField(const WbBaseNode *node, WbValue *value) {
  foreach (const WbField *field, node->fields())
    if (field->value() == value)
      return field;
  return NULL;
}
