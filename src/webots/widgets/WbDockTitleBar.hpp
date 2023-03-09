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

#ifndef WB_DOCK_TITLE_BAR_HPP
#define WB_DOCK_TITLE_BAR_HPP

//
// Description: title bar for WbDockWidget and WbSimulationView
//   with a text label and "minimize"/"maximize" + "float/dock" + "close" buttons
//

class QLabel;
class QPushButton;

#include <QtGui/QIcon>
#include <QtWidgets/QFrame>

class WbDockTitleBar : public QFrame {
  Q_OBJECT

public:
  explicit WbDockTitleBar(bool hasFloatButton, QWidget *parent = NULL);
  virtual ~WbDockTitleBar();

  // text label that appears in the dock's toolbar
  void setTitle(const QString &title);

  // just change the button's icons
  void setMaximized(bool maximized);
  bool isMaximized() const { return mMaximized; }

  // hide the "minimize"/"maximize" button
public slots:
  void setFloating(bool floating);

signals:
  // signals called when the corresponding toolbar buttons are pushed
  void closeClicked();
  void floatClicked();
  void maximizeClicked();
  void minimizeClicked();

protected:
  void resizeEvent(QResizeEvent *event) override;

private:
  void updateTitle();
  int computeNumberOfButtons();

  QString mTitle;
  bool mMaximized;
  QLabel *mTitleLabel;
  QPushButton *mToggleButton, *mCloseButton, *mFloatButton;
  QIcon mMaximizeIcon, mMinimizeIcon;

private slots:
  void toggle();
};

#endif
