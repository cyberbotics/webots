// Copyright 1996-2020 Cyberbotics Ltd.
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

#ifndef WB_MULTI_SELECTION_DIALOG_HPP
#define WB_MULTI_SELECTION_DIALOG_HPP

//
// Description: a simple input dialog allowing supporting multi selection
//

#include <QtCore/QVector>
#include <QtWidgets/QDialog>

class QCheckBox;

class WbMultiSelectionDialog : public QDialog {
  Q_OBJECT

public:
  explicit WbMultiSelectionDialog(const QString &description, const QStringList &options, QWidget *parent = 0);
  virtual ~WbMultiSelectionDialog() {}

  const QStringList &enabledOptions() const { return mEnabledOptions; }

private slots:
  void validate();

private:
  QStringList mEnabledOptions;
  QVector<QCheckBox *> mCheckboxes;
};

#endif
