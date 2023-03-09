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

#include "WbDockWidget.hpp"

#include "WbDockTitleBar.hpp"

WbDockWidget::WbDockWidget(QWidget *parent) : QDockWidget(parent) {
  mTitleBar = new WbDockTitleBar(true, this);
  connect(mTitleBar, &WbDockTitleBar::closeClicked, this, &WbDockWidget::close);
  connect(mTitleBar, &WbDockTitleBar::maximizeClicked, this, &WbDockWidget::needsMaximize);
  connect(mTitleBar, &WbDockTitleBar::minimizeClicked, this, &WbDockWidget::needsMinimize);
  connect(mTitleBar, &WbDockTitleBar::floatClicked, this, &WbDockWidget::makeFloat);
  connect(this, &WbDockWidget::topLevelChanged, mTitleBar, &WbDockTitleBar::setFloating);

  setTitleBarWidget(mTitleBar);
}

WbDockWidget::~WbDockWidget() {
}

void WbDockWidget::setWindowTitle(const QString &title) {
  mTitleBar->setTitle(title);
}

void WbDockWidget::setTabbedTitle(const QString &title) {
  QDockWidget::setWindowTitle(title);
}

void WbDockWidget::setMaximized(bool maximized) {
  if (maximized)
    setFeatures(QDockWidget::NoDockWidgetFeatures);
  else
    setFeatures(DockWidgetClosable | DockWidgetMovable | DockWidgetFloatable);

  mTitleBar->setMaximized(maximized);
}

bool WbDockWidget::isMaximized() const {
  return mTitleBar->isMaximized();
}

void WbDockWidget::makeFloat() {
#ifdef __APPLE__
  // otherwise the dockWidget titlebar is sometimes
  // (when clicked on the float button) hidden
  // behind the OS menu bar
  if (pos().y() <= 0)
    move(pos().x(), pos().y() + 50);
#endif
  setFloating(!isFloating());
}
