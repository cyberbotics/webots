// Copyright 1996-2022 Cyberbotics Ltd.
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
class WbWriter;

namespace WbUrl {
  QString resolveUrl(const QString &rawUrl);
  QString computePath(const WbNode *node, const QString &field, const QString &rawUrl);
  QString computePath(const WbNode *node, const QString &field, const WbMFString *urlField, int index);

  QString combinePaths(const QString &rawUrl, const QString &rawParentUrl);

  QString exportResource(const WbNode *node, const QString &url, const QString &sourcePath, const QString &relativeResourcePath,
                         const WbWriter &writer, const bool isTexture = true);
  QString exportTexture(const WbNode *node, const WbMFString *urlField, int index, const WbWriter &writer);
  QString exportMesh(const WbNode *node, const WbMFString *urlField, int index, const WbWriter &writer);

  const QString missing(const QString &url);
  const QString &missingTexture();
  const QString &missingProtoIcon();
  bool isWeb(const QString &url);
  bool isLocalUrl(const QString &url);
  const QString computeLocalAssetUrl(QString url, bool isX3d);
  const QString computePrefix(const QString &rawUrl);

  const QString remoteWebotsAssetRegex(bool capturing);
  const QString &remoteWebotsAssetPrefix();

  QString expressRelativeToWorld(const QString &url);
};  // namespace WbUrl

#endif
