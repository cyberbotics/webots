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

#ifndef WB_URL_HPP
#define WB_URL_HPP

#include <QtCore/QString>

class WbNode;
class WbMFString;
class WbVrmlWriter;

namespace WbUrl {
  // return search path ordered by decreasing priority
  QStringList orderedSearchPaths(const WbNode *node);
  QString computePath(const WbNode *node, const QString &field, const QString &url, bool displayWarning = true);
  QString computePath(const WbNode *node, const QString &field, const WbMFString *urlField, int index);
  QString exportTexture(const WbNode *node, const QString &url, const QString &sourcePath, const QString &relativeTexturesPath,
                        const WbVrmlWriter &writer);
  QString exportTexture(const WbNode *node, const WbMFString *urlField, int index, const WbVrmlWriter &writer);
};  // namespace WbUrl

#endif
