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

#ifndef WB_VELOCITY_VIEWER_HPP
#define WB_VELOCITY_VIEWER_HPP

//
// Description: viewer showing information about Solid's velocity
//

#include <QtWidgets/QWidget>

class QVBoxLayout;
class QComboBox;
class QLabel;
class WbSolid;

class WbVelocityViewer : public QWidget {
  Q_OBJECT

public:
  explicit WbVelocityViewer(QWidget *parent = NULL);
  virtual ~WbVelocityViewer();

  void show(WbSolid *solid);

  void stopUpdating();
  void setSelected(bool selected);
  void triggerPhysicsUpdates();

public slots:
  void clean();
  void update();
  void requestUpdate();

private:
  void updateRelativeToComboBox();

  WbSolid *mSolid;
  bool mIsSelected;

  // relative to boxes
  QComboBox *mRelativeToComboBox;

  // Layouts
  QVBoxLayout *mTopLayout;

  // Labels
  QVector<QLabel *> mLinearVelocityLabels;
  QVector<QLabel *> mAngularVelocityLabels;

private slots:
  void updateRelativeTo(int index);
};

#endif
