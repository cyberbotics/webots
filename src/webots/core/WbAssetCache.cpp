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

#include "WbAssetCache.hpp"
#include "WbPreferences.hpp"

#include <QtCore/QDir>
#include <QtCore/QStandardPaths>

static WbAssetCache *gInstance = NULL;

void WbAssetCache::cleanup() {
  delete gInstance;
}

WbAssetCache *WbAssetCache::instance() {
  if (gInstance == NULL)
    gInstance = new WbAssetCache();
  return gInstance;
}

WbAssetCache::WbAssetCache() {
  QDir dir(QStandardPaths::writableLocation(QStandardPaths::CacheLocation) + "/assets");
  if (!dir.exists())
    dir.mkpath(".");
}

WbAssetCache::~WbAssetCache() {
  gInstance = NULL;
}

int WbAssetCache::size() {
  // int value = 1024 * 1024 * WbPreferences::instance()->value("Network/cacheSize", 1024).toInt();
  return 0;
}

void WbAssetCache::clearCache() {
}
