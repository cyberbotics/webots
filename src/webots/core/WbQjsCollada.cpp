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
#include "WbLog.hpp"
#include "WbNodeOperations.hpp"
#include "WbStandardPaths.hpp"

QString WbQjsCollada::getVrmlFromFile(const QString &filePath) {
  QString stream = "";
  WbNodeOperations::OperationResult result =
    WbNodeOperations::instance()->getVrmlFromExternalModel(stream, filePath, true, true, true, false, false, true);
  if (result == WbNodeOperations::OperationResult::FAILURE) {
    WbLog::instance()->error(QString("JavaScript error: cannot parse the Collada file: %1.").arg(filePath), false,
                             WbLog::PARSING);
    return "";
  }
  return stream;
}
