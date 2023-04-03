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

#ifndef WB_DOCK_WIDGET_HPP
#define WB_DOCK_WIDGET_HPP

//
// Description: base class for all dock widgets in Webots
//
// Inherited by:
//   WbTextEditor, WbSceneTree, WbConsole
//

class WbDockTitleBar;

#include <QtGui/QIcon>
#include <QtWidgets/QDockWidget>

class WbDockWidget : public QDockWidget {
  Q_OBJECT

public:
  explicit WbDockWidget(QWidget *parent = NULL);
  virtual ~WbDockWidget();

  // text label that appears in the dock's toolbar
  void setWindowTitle(const QString &title);

  // set label that appears when the dock is tabbed
  void setTabbedTitle(const QString &title);

  // just change the icons appearance
  void setMaximized(bool maximized);
  bool isMaximized() const;

signals:
  // signals called when the corresponding toolbar buttons are pushed
  void needsMaximize();
  void needsMinimize();

private:
  WbDockTitleBar *mTitleBar;

private slots:
  void makeFloat();
};

#endif
