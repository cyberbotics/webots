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

#include "BotStudioPaths.hpp"

#include <core/StandardPaths.hpp>

#include <QtCore/QDir>

using namespace webotsQtUtils;

const QString &BotStudioPaths::getBotStudioPath() {
  static QString path(StandardPaths::getWebotsHomePath() + "projects/robots/gctronic/e-puck/plugins/robot_windows/botstudio/");
  return path;
};

const QString &BotStudioPaths::getIconsPath() {
  static QString path(getBotStudioPath() + "icons/");
  return path;
};
