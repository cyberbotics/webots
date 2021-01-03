// Copyright 1996-2021 Cyberbotics Ltd.
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

//
//  WbTrack.hpp
//

// Implemented node class representing a wheel of the WbTrack node
//   special WbTransform node where 'translation', 'rotation, 'scale',
//   'translationStep' and 'rotationStep' fields are not open to the user
//    but defined internally

#ifndef WB_TRACK_WHEEL_HPP
#define WB_TRACK_WHEEL_HPP

#include "WbSFBool.hpp"
#include "WbSFDouble.hpp"
#include "WbSFVector2.hpp"
#include "WbTransform.hpp"

class WbTrackWheel : public WbTransform {
  Q_OBJECT
public:
  explicit WbTrackWheel(WbTokenizer *tokenizer = NULL);
  WbTrackWheel(const WbTrackWheel &other);
  explicit WbTrackWheel(const WbNode &other);
  virtual ~WbTrackWheel();

  // reimplemented public functions
  int nodeType() const override { return WB_NODE_TRACK_WHEEL; }
  void preFinalize() override;
  void postFinalize() override;
  void write(WbVrmlWriter &writer) const override;
  void exportNodeFields(WbVrmlWriter &writer) const override;

  const WbVector2 position() const { return mPosition->value(); }
  double radius() const { return mRadius->value(); }
  bool inner() const { return mInner->value(); }

  void rotate(double travelledDistance);

signals:
  void changed();

protected:
  const QString &vrmlName() const override {
    static const QString name("Transform");
    return name;
  }

private:
  WbTrackWheel &operator=(const WbTrackWheel &);  // non copyable
  WbNode *clone() const override { return new WbTrackWheel(*this); }
  void init();

  WbSFVector2 *mPosition;
  WbSFDouble *mRadius;
  WbSFBool *mInner;

private slots:
  void updatePosition();
  void updateRadius();
};

#endif
