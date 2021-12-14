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

#include "WbQjsCollada.hpp"
#include "WbStandardPaths.hpp"

WbQjsCollada *WbQjsCollada::cInstance = NULL;
QString WbQjsCollada::cVrmlResponse = "";

WbQjsCollada *WbQjsCollada::instance() {
  if (WbQjsCollada::cInstance == NULL)
    WbQjsCollada::cInstance = new WbQjsCollada();
  return WbQjsCollada::cInstance;
}

QString WbQjsCollada::getVrmlFromFile(const QString &filePath) {
  emit WbQjsCollada::instance()->vrmlFromFileRequested(filePath);
  return WbQjsCollada::cVrmlResponse;
}
