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

#ifndef WB_FIELD_DOUBLE_SPIN_BOX_HPP
#define WB_FIELD_DOUBLE_SPIN_BOX_HPP

//
// Description: double spin box specialized for:
//  -increasing decreasing only by the last decimal
//  -handling axis/angle rotation components
//  -handling RGB color components
//

#include "WbDoubleSpinBox.hpp"

class QKeyEvent;
class QFocusEvent;

class WbFieldDoubleSpinBox : public WbDoubleSpinBox {
  Q_OBJECT

public:
  enum { NORMAL, RADIANS, AXIS, RGB };
  explicit WbFieldDoubleSpinBox(QWidget *parent = NULL, int mode = NORMAL);
  virtual ~WbFieldDoubleSpinBox();

  void setValueNoSignals(double value);
  void setMode(int mode);

  // reimplemented public functions
  QString textFromValue(double value) const override;
  void stepBy(int steps) override;

signals:
  void valueApplied();
  void focusLeft();

protected:
  void keyPressEvent(QKeyEvent *event) override;
  void keyReleaseEvent(QKeyEvent *event) override;
  void focusOutEvent(QFocusEvent *event) override;

private slots:
  void findDecimals(const QString &text);

private:
  int mDecimals;
  int mMode;
};

#endif
