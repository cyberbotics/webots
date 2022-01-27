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

#ifndef WB_MASS_CHECKER_HPP
#define WB_MASS_CHECKER_HPP

#include <QtCore/QObject>

class WbMassChecker : public QObject {
  Q_OBJECT

public:
  static WbMassChecker *instance();

public slots:
  void checkMasses();

private:
  static void cleanup();

  WbMassChecker() {}
  virtual ~WbMassChecker() {}

  static WbMassChecker *cInstance;
};

#endif
