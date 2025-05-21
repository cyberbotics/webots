// Copyright 1996-2025 Cyberbotics Ltd.
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

#include "WbNodeFactory.hpp"
#include <QDebug> // For qDebug

static WbNodeFactory *gInstance = 0;

WbNodeFactory *WbNodeFactory::instance() {
  qDebug() << "WbNodeFactory::instance() called, returning gInstance:" << gInstance;
  return gInstance;
}

WbNodeFactory::WbNodeFactory() {
  qDebug() << "WbNodeFactory CONSTRUCTOR (this:" << this << "). Setting gInstance.";
  gInstance = this;
  qDebug() << "WbNodeFactory CONSTRUCTOR: gInstance is now:" << gInstance;
}

WbNodeFactory::~WbNodeFactory() {
  qDebug() << "WbNodeFactory DESTRUCTOR (this:" << this << "). Setting gInstance to 0.";
  if (gInstance == this) // Only nullify if it's us
    gInstance = 0;
  else
    qDebug() << "WbNodeFactory DESTRUCTOR (this:" << this << "): gInstance was already" << gInstance << ", not this instance.";
  qDebug() << "WbNodeFactory DESTRUCTOR: gInstance is now:" << gInstance;
}
