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

#ifndef WB_SOURCE_FILE_EDITOR_HPP
#define WB_SOURCE_FILE_EDITOR_HPP

//
// Description: edits source files in the WbTextEditor
//

#include "WbStringEditor.hpp"

class QPushButton;

class WbExtendedStringEditor : public WbStringEditor {
  Q_OBJECT

public:
  explicit WbExtendedStringEditor(QWidget *parent = NULL);
  virtual ~WbExtendedStringEditor();

  void recursiveBlockSignals(bool block) override;

protected:
  void edit(bool copyOriginalValue) override;
  void resetFocus() override;

signals:
  void editRequested(const QString &fileName);

private:
  // type of field currently edited
  enum StringType {
    CONTROLLER,
    FLUID_NAME,
    PHYSICS_PLUGIN,
    REFERENCE_AREA,
    REMOTE_CONTROL_PLUGIN,
    ROBOT_WINDOW_PLUGIN,
    SOLID_REFERENCE,
    // No info from there.
    N_STRING_TYPE_INFO,  // counter

    REGULAR,
    SOUND,
    TEXTURE_URL,
    HDR_TEXTURE_URL,
    MESH_URL,
    CAD_URL,
    SKIN_URL
  };
  StringType mStringType;

  bool isWorldInfoPluginType(StringType type) const;

  // optional buttons below the string field
  QPushButton *mSelectButton, *mEditButton;

  // default system search directories
  mutable QStringList mDefaultControllersEntryList;
  mutable QStringList mDefaultRobotWindowPluginsEntryList;
  mutable QStringList mDefaultRemoteControlPluginsEntryList;
  mutable QStringList mDefaultPhysicsPluginsEntryList;

  const QStringList &defaultControllersEntryList() const;
  const QStringList &defaultRobotWindowsPluginsEntryList() const;
  const QStringList &defaultRemoteControlsPluginsEntryList() const;
  const QStringList &defaultPhysicsPluginsEntryList() const;
  const QStringList &defaultEntryList() const;
  void updateWidgets();

  void selectFile(const QString &folder, const QString &title, const QString &types);

  // selection of a solid reference / a fluid name / a reference area
  bool selectItem();
  bool populateItems(QStringList &items);

  static StringType fieldNameToStringType(const QString &fieldName, const WbNode *parentNode);
  static const QStringList ITEM_LIST_INFO[N_STRING_TYPE_INFO];
  static const QStringList REFERENCE_AREA_ITEMS;

private slots:
  void editInTextEditor();
  void select();
};

#endif
