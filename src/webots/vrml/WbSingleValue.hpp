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

#ifndef WB_SINGLE_VALUE_HPP
#define WB_SINGLE_VALUE_HPP

//
// Description: abstract base class for single field values
//
// Inherited by:
//   WbSFVector3, WbSFVector2, WbSFString, WbSFRotation, WbSFNode, WbSFInt, WbSFDouble, WbSFColor, WBSFBool
//

#include "WbValue.hpp"
#include "WbVariant.hpp"

class WbSingleValue : public WbValue {
  Q_OBJECT

public:
  virtual ~WbSingleValue();

  // return generic value
  virtual WbVariant variantValue() const = 0;
  QString toString(WbPrecision::Level level = WbPrecision::DOUBLE_MAX) const override {
    return variantValue().toSimplifiedStringRepresentation(level);
  }

protected:
  WbSingleValue();

private:
};

#endif
