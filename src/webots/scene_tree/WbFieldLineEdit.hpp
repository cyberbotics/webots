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

#ifndef WB_FIELD_LINE_EDIT_HPP
#define WB_FIELD_LINE_EDIT_HPP

//
// Description: line edit with a slighlty modified signal handling
//

#include "WbLineEdit.hpp"

class WbFieldLineEdit : public WbLineEdit {
  Q_OBJECT

public:
  explicit WbFieldLineEdit(QWidget *parent = 0);
  virtual ~WbFieldLineEdit();

signals:
  void focusLeft();

protected:
  void keyPressEvent(QKeyEvent *event) override;
  void focusOutEvent(QFocusEvent *event) override;
};

#endif
