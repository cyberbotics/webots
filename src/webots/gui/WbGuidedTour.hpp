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

#ifndef WB_GUIDED_TOUR_HPP
#define WB_GUIDED_TOUR_HPP

//
// Description: Webots Guided Tour dialog
//

#include <QtWidgets/QDialog>

class QLabel;
class QPlainTextEdit;
class QCheckBox;
class QPushButton;
class QTimer;
class QTreeWidget;
class QTreeWidgetItem;

class WbGuidedTour : public QDialog {
  Q_OBJECT

public:
  // get or create the singleton WbGuidedTour and center on parent
  static WbGuidedTour *instance(QWidget *parent);
  static bool set(const QString &name);

signals:
  void loadWorldRequest(const QString &fileName);

protected:
private slots:
  void prev();
  void next();
  void shoot();
  void setSimulationDeadline(bool autoChecked);
  void selectWorld();
  void worldLoaded();
  void updateGUI();

private:
  QVector<QString> mFilenames;
  QVector<int> mDurations;
  QVector<QTreeWidgetItem *> mWorlds;
  QVector<QTreeWidgetItem *> mSections;
  int mIndex;
  bool mReady;
  QTimer *mTimer;
  double mDeadline;
  QTreeWidget *mTree;
  QLabel *mTitleLabel;
  QPlainTextEdit *mInfoText;
  QCheckBox *mAutoBox;
  QPushButton *mPrevButton;
  QPushButton *mNextButton;
  explicit WbGuidedTour(QWidget *parent);
  ~WbGuidedTour();
  void loadWorld();
  void nextWorld();
  void loadWorldList();
  void selectCurrent();
  void setTitleText(const QString &title);
};

#endif
