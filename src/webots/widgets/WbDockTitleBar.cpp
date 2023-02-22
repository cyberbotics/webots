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

#include "WbDockTitleBar.hpp"

#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QStyle>

static QPushButton *createSmallPushButton(QIcon icon, QWidget *parent) {
  QPushButton *button = new QPushButton(icon, "", parent);
  button->setObjectName("dockButton");

  // avoid keyboard focus, this looks bad
  button->setFocusPolicy(Qt::NoFocus);

  return button;
}

WbDockTitleBar::WbDockTitleBar(bool hasFloatButton, QWidget *parent) : QFrame(parent) {
  if (hasFloatButton)
    setObjectName("floatableDockTitleFrame");

  mMaximized = false;

  // title bar horizontal layout
  QHBoxLayout *layout = new QHBoxLayout(this);

  // ugly platform dependent layout fix:
  // this should be solved by the style sheet
  layout->setContentsMargins(10, 3, 10, 3);  // left, top, right, bottom
#ifdef __APPLE__
  layout->setSpacing(10);
#else
  layout->setSpacing(0);
#endif

  mTitle = "";
  mTitleLabel = new QLabel(mTitle, this);
  mTitleLabel->setMinimumWidth(10);
  mTitleLabel->setObjectName("dockWidgetTitleText");
  mTitleLabel->setAlignment(Qt::AlignRight | Qt::AlignVCenter);

  // title bar icons
  mMaximizeIcon = QIcon("enabledIcons:dock_maximize_button.png");
  mMinimizeIcon = QIcon("enabledIcons:dock_minimize_button.png");
  QIcon closeIcon("enabledIcons:dock_close_button.png");

  mToggleButton = createSmallPushButton(mMaximizeIcon, this);
  mToggleButton->setToolTip(tr("Maximize the panel."));
  mCloseButton = createSmallPushButton(closeIcon, this);
  mCloseButton->setObjectName("dockCloseButton");
  mCloseButton->setToolTip(tr("Close the panel."));

  if (hasFloatButton) {
    QIcon floatIcon("enabledIcons:dock_float_button.png");
    mFloatButton = createSmallPushButton(floatIcon, this);
    mFloatButton->setToolTip(tr("Undock the panel."));
    connect(mFloatButton, &QPushButton::clicked, this, &WbDockTitleBar::floatClicked);
  } else
    mFloatButton = NULL;

#ifdef __APPLE__  // respect the Mac standard layout
  // from left to right
  layout->addWidget(mCloseButton);
  if (mFloatButton)
    layout->addWidget(mFloatButton);
  layout->addWidget(mToggleButton);

  layout->addWidget(mTitleLabel);
  layout->addStretch();
#else
  // from left to right
  layout->addWidget(mTitleLabel);

  layout->addStretch();

  layout->addWidget(mToggleButton);
  if (mFloatButton)
    layout->addWidget(mFloatButton);
  layout->addWidget(mCloseButton);
#endif

  connect(mCloseButton, &QPushButton::clicked, this, &WbDockTitleBar::closeClicked);
  connect(mToggleButton, &QPushButton::clicked, this, &WbDockTitleBar::toggle);
}

WbDockTitleBar::~WbDockTitleBar() {
}

void WbDockTitleBar::setTitle(const QString &title) {
  mTitle = title;
  updateTitle();
}

void WbDockTitleBar::setMaximized(bool maximized) {
  mMaximized = maximized;

  if (maximized) {
    mToggleButton->setIcon(mMinimizeIcon);
    mToggleButton->setToolTip(tr("Minimize the panel."));
  } else {
    mToggleButton->setIcon(mMaximizeIcon);
    mToggleButton->setToolTip(tr("Maximize the panel."));
  }
  if (mFloatButton)
    mFloatButton->setVisible(!maximized);

  mCloseButton->setVisible(!maximized);
  updateTitle();
}

void WbDockTitleBar::toggle() {
  if (mMaximized)
    emit minimizeClicked();
  else
    emit maximizeClicked();
}

void WbDockTitleBar::setFloating(bool floating) {
  mToggleButton->setVisible(!floating);
  updateTitle();
}

void WbDockTitleBar::resizeEvent(QResizeEvent *event) {
  updateTitle();
}

int WbDockTitleBar::computeNumberOfButtons() {
  int buttons = 0;
  if (mToggleButton && mToggleButton->isVisible())
    buttons++;
  if (mCloseButton && mCloseButton->isVisible())
    buttons++;
  if (mFloatButton && mFloatButton->isVisible())
    buttons++;
  return buttons;
}

void WbDockTitleBar::updateTitle() {
  QString title = mTitle;
  const QFontMetrics metrics(mTitleLabel->font());

  const int maxSize = size().width() - ((mToggleButton->size().width() + 12) * computeNumberOfButtons());

  while (metrics.horizontalAdvance(title) > maxSize && title.size() > 3) {
    title.remove(0, 1);
    title.replace(0, 3, "...");
  }

  mTitleLabel->setText(title);
}
