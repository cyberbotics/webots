// Copyright 1996-2022 Cyberbotics Ltd.
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

#include "WbNewProtoWizard.hpp"

#include "WbApplicationInfo.hpp"
#include "WbFieldModel.hpp"
#include "WbFileUtil.hpp"
#include "WbLineEdit.hpp"
#include "WbLog.hpp"
#include "WbMessageBox.hpp"
#include "WbMultipleValue.hpp"
#include "WbNodeModel.hpp"
#include "WbPreferences.hpp"
#include "WbProject.hpp"
#include "WbProtoManager.hpp"
#include "WbProtoModel.hpp"
#include "WbStandardPaths.hpp"
#include "WbVersion.hpp"

#include <QtCore/QDirIterator>
#include <QtGui/QRegularExpressionValidator>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QLabel>
#include <QtWidgets/QScrollArea>
#include <QtWidgets/QTreeWidgetItem>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWizardPage>

enum { INTRO, NAME, TAGS, BASE_TYPE, CONCLUSION };
enum { BASE_NODE = 10001, PROTO_WORLD = 10002, PROTO_PROJECT = 10003, PROTO_EXTRA = 10004, PROTO_WEBOTS = 10005 };

static const QStringList defaultFields = {"translation", "rotation", "name", "controller"};

WbNewProtoWizard::WbNewProtoWizard(QWidget *parent) : QWizard(parent) {
  mNeedsEdit = false;

  addPage(createIntroPage());
  addPage(createNamePage());
  addPage(createTagsPage());
  addPage(createBaseTypeSelectorPage());
  addPage(createConclusionPage());

  setOption(QWizard::NoCancelButton, false);
  setOption(QWizard::CancelButtonOnLeft, true);
  setWindowTitle(tr("Create a new PROTO"));

  setMinimumSize(600, 400);
}

void WbNewProtoWizard::updateUI() {
  // update paths
  mProtoDir = WbProject::current()->protosPath();
  mProtoFullPath = mProtoDir + mNameEdit->text() + ".proto";

  // update check box message
  mEditCheckBox->setText(tr("Open '%1.proto' in Text Editor.").arg(mNameEdit->text()));

  QString files = mProtoDir + "\n" + mProtoFullPath;
  mFilesLabel->setText(QDir::toNativeSeparators(files));
}

bool WbNewProtoWizard::validateCurrentPage() {
  updateUI();

  if (currentId() == NAME)
    return !mNameEdit->text().isEmpty();

  return true;
}

void WbNewProtoWizard::accept() {
  // create protos directory
  bool success = QDir::root().mkpath(mProtoDir);

  // copy PROTO from template and rename
  const QString src = WbStandardPaths::templatesPath() + "protos/template.proto";
  success = WbFileUtil::copyAndReplaceString(src, mProtoFullPath, "template", mNameEdit->text()) && success;

  if (success) {
    QFile file(protoName());
    if (!file.open(QIODevice::ReadWrite)) {
      WbMessageBox::warning(tr("PROTO template not found."), this, tr("PROTO creation failed"));
      return;
    }

    QByteArray protoContent = file.readAll();

    QString tags;
    if (mProceduralCheckBox->isChecked())
      tags += "# template language: javascript\n";

    if (mNonDeterministicCheckbox->isChecked() || mHiddenCheckBox->isChecked()) {
      tags += "# tags: ";

      if (mNonDeterministicCheckbox->isChecked())
        tags += "nonDeterministic, ";
      if (mHiddenCheckBox->isChecked())
        tags += "hidden, ";

      tags.chop(2);
    }

    tags += "\n";  // leave one empty line before the proto definition

    QString parameters = "";
    QString body = "";

    QList<WbFieldModel *> fieldModels;
    // if base node was selected, define exposed parameters and PROTO body accordingly
    if (mBaseNode != "") {
      if (mIsProtoNode) {
        WbProtoModel *protoModel = WbProtoManager::instance()->customFindModel(mBaseNode, "");
        assert(protoModel);
        fieldModels = protoModel->fieldModels();
      } else {
        WbNodeModel *nodeModel = WbNodeModel::findModel(mBaseNode);
        fieldModels = nodeModel->fieldModels();
      }

      assert(mExposedFieldCheckBoxes.size() - 1 == fieldModels.size());  // extra entry is "expose all" checkbox

      for (int i = 0; i < fieldModels.size(); ++i) {
        if (mExposedFieldCheckBoxes[i + 1]->isChecked()) {
          const WbValue *defaultValue = fieldModels[i]->defaultValue();
          QString vrmlDefaultValue = defaultValue->toString();

          if (defaultValue->type() == WB_SF_NODE && vrmlDefaultValue != "NULL")
            vrmlDefaultValue += "{}";

          const WbMultipleValue *mv = dynamic_cast<const WbMultipleValue *>(defaultValue);
          if (defaultValue->type() == WB_MF_NODE && mv) {
            vrmlDefaultValue = "[ ";
            for (int j = 0; j < mv->size(); ++j)
              vrmlDefaultValue += mv->itemToString(j) + "{} ";
            vrmlDefaultValue += "]";
          }
          parameters +=
            "  field " + defaultValue->vrmlTypeName() + " " + fieldModels[i]->name() + " " + vrmlDefaultValue + "\n";
        }
      }

      parameters.chop(1);  // chop new line

      body += "  " + mBaseNode + " {\n";
      for (int i = 0; i < fieldModels.size(); ++i)
        if (mExposedFieldCheckBoxes[i + 1]->isChecked())
          body += "    " + fieldModels[i]->name() + " IS " + fieldModels[i]->name() + "\n";
      body += "  }";
    }

    const QString release = WbApplicationInfo::version().toString(false);
    QString description = mDescription->toPlainText();
    description.insert(0, "# ");
#ifdef _WIN32
    description.replace("\r\n", "\n");
#endif
    description.replace("\n", "\n# ");  // prepend a '#' before every new line

    protoContent.replace(QByteArray("%description%"), description.toUtf8());
    protoContent.replace(QByteArray("%tags%"), tags.toUtf8());
    protoContent.replace(QByteArray("%name%"), mNameEdit->text().toUtf8());
    protoContent.replace(QByteArray("%release%"), release.toUtf8());
    protoContent.replace(QByteArray("%body%"), body.toUtf8());
    protoContent.replace(QByteArray("%parameters%"), parameters.toUtf8());

    file.seek(0);
    file.write(protoContent);
    file.resize(file.pos());
    file.close();
  }

  if (!success)
    WbMessageBox::warning(tr("Some directories or files could not be created."), this, tr("PROTO creation failed"));

  mNeedsEdit = mEditCheckBox->isChecked();

  QDialog::accept();
}

bool WbNewProtoWizard::needsEdit() const {
  return mNeedsEdit;
}

const QString &WbNewProtoWizard::protoName() const {
  return mProtoFullPath;
}

QWizardPage *WbNewProtoWizard::createIntroPage() {
  QWizardPage *page = new QWizardPage(this);

  page->setTitle(tr("New PROTO creation"));

  QVBoxLayout *layout = new QVBoxLayout(page);
  QLabel *label = new QLabel(tr("This wizard will help you creating a new PROTO."), page);
  layout->addWidget(label);

  return page;
}

QWizardPage *WbNewProtoWizard::createNamePage() {
  QWizardPage *page = new QWizardPage(this);

  page->setTitle(tr("Name selection"));
  page->setSubTitle(tr("Please choose a name for your PROTO node."));

  QLabel *nameLabel = new QLabel(tr("PROTO name:"), page);
  mNameEdit = new WbLineEdit("MyProto", page);
  mNameEdit->setValidator(new QRegularExpressionValidator(QRegularExpression("[a-zA-Z0-9_-]*"), page));
  nameLabel->setBuddy(mNameEdit);

  mDescription = new QPlainTextEdit(tr("Describe the functionality of your PROTO here."));
  mDescription->setObjectName("ProtoDescription");

  QVBoxLayout *layout = new QVBoxLayout(page);
  QHBoxLayout *nameLayout = new QHBoxLayout();
  nameLayout->addWidget(nameLabel);
  nameLayout->addWidget(mNameEdit);
  layout->addLayout(nameLayout);
  layout->addWidget(mDescription);

  return page;
}

QWizardPage *WbNewProtoWizard::createTagsPage() {
  QWizardPage *page = new QWizardPage(this);

  page->setTitle(tr("Tags selection"));
  page->setSubTitle(tr("Please choose the tags of your PROTO."));

  QVBoxLayout *layout = new QVBoxLayout(page);

  mHiddenCheckBox = new QCheckBox(page);
  mNonDeterministicCheckbox = new QCheckBox(page);
  mProceduralCheckBox = new QCheckBox(page);

  QLabel *nonDeterministicTagDescription = new QLabel(page);
  QLabel *proceduralTagDescription = new QLabel(page);
  QLabel *hiddenTagDescription = new QLabel(page);

  mProceduralCheckBox->setChecked(false);
  mProceduralCheckBox->setText(tr("Procedural PROTO"));
  proceduralTagDescription->setText("<i>" +
                                    tr("By enabling this option, JavaScript template scripting can be used\n"
                                       "to generate PROTO in a procedural way.") +
                                    "</i>");
  proceduralTagDescription->setWordWrap(true);
  proceduralTagDescription->setIndent(20);

  mNonDeterministicCheckbox->setChecked(false);
  mNonDeterministicCheckbox->setText(tr("Non-deterministic PROTO"));
  nonDeterministicTagDescription->setText("<i>" +
                                          tr("A non-deterministic PROTO is a PROTO where the same fields can potentially\n"
                                             "yield a different result from run to run. This is often the case if random\n"
                                             "number generators with time-based seeds are employed.") +
                                          "</i>");
  nonDeterministicTagDescription->setWordWrap(true);
  nonDeterministicTagDescription->setIndent(20);

  mHiddenCheckBox->setChecked(false);
  mHiddenCheckBox->setText(tr("Hidden PROTO"));
  hiddenTagDescription->setText("<i>" +
                                tr("A hidden PROTO will not appear in the list when adding a new node.\n"
                                   "This tag is often used for sub-PROTO, when creating components of\n"
                                   "a larger node.") +
                                "</i>");
  hiddenTagDescription->setWordWrap(true);
  hiddenTagDescription->setIndent(20);

  layout->setSpacing(4);
  layout->addWidget(mProceduralCheckBox);
  layout->addWidget(proceduralTagDescription);
  layout->addWidget(mNonDeterministicCheckbox);
  layout->addWidget(nonDeterministicTagDescription);
  layout->addWidget(mHiddenCheckBox);
  layout->addWidget(hiddenTagDescription);

  return page;
}

QWizardPage *WbNewProtoWizard::createBaseTypeSelectorPage() {
  QWizardPage *page = new QWizardPage(this);

  page->setTitle(tr("Base type selection"));
  page->setSubTitle(tr("Please choose the base type from which the PROTO will inherit."));

  QHBoxLayout *const mainLayout = new QHBoxLayout(page);
  QVBoxLayout *const nodeListLayout = new QVBoxLayout();
  QVBoxLayout *const fieldListLayout = new QVBoxLayout();

  QFont font;
  font.fromString(WbPreferences::instance()->value("Editor/font").toString());
  mFindLineEdit = new QLineEdit(this);
  mFindLineEdit->setFont(font);
  mFindLineEdit->setClearButtonEnabled(true);

  mTree = new QTreeWidget();
  mFields = new QWidget(this);

  nodeListLayout->addWidget(mFindLineEdit);
  nodeListLayout->addWidget(mTree);
  fieldListLayout->addWidget(mFields);

  mainLayout->addLayout(nodeListLayout);
  mainLayout->addLayout(fieldListLayout);

  connect(mFindLineEdit, &QLineEdit::textChanged, this, &WbNewProtoWizard::updateNodeTree);
  connect(mTree, &QTreeWidget::itemSelectionChanged, this, &WbNewProtoWizard::updateBaseNode);

  updateNodeTree();

  return page;
}

void WbNewProtoWizard::updateNodeTree() {
  mTree->clear();
  mTree->setHeaderHidden(true);

  QTreeWidgetItem *const nodesItem = new QTreeWidgetItem(QStringList(tr("Base nodes")), BASE_NODE);
  QTreeWidgetItem *const worldProtosItem = new QTreeWidgetItem(QStringList("PROTO nodes (Current World File)"), PROTO_WORLD);
  QTreeWidgetItem *const projectProtosItem = new QTreeWidgetItem(QStringList("PROTO nodes (Current Project)"), PROTO_PROJECT);
  QTreeWidgetItem *const extraProtosItem = new QTreeWidgetItem(QStringList(tr("PROTO nodes (Extra Projects)")), PROTO_EXTRA);
  QTreeWidgetItem *const webotsProtosItem = new QTreeWidgetItem(QStringList("PROTO nodes (Webots Projects)"), PROTO_WEBOTS);

  // list of all available base nodes
  const QStringList nodes = WbNodeModel::baseModelNames();
  const QRegularExpression regexp(
    QRegularExpression::wildcardToRegularExpression(mFindLineEdit->text(), QRegularExpression::UnanchoredWildcardConversion),
    QRegularExpression::CaseInsensitiveOption);

  // list of all available base nodes
  foreach (const QString &basicNodeName, nodes) {
    QFileInfo fileInfo(basicNodeName);
    if (fileInfo.baseName().contains(regexp))
      nodesItem->addChild(new QTreeWidgetItem(nodesItem, QStringList(fileInfo.baseName())));
  }
  // list of all available protos in the current world file
  foreach (const QString &protoName, WbProtoManager::instance()->nameList(WbProtoManager::PROTO_WORLD)) {
    if (protoName.contains(regexp))
      worldProtosItem->addChild(new QTreeWidgetItem(worldProtosItem, QStringList(protoName)));
  }
  // list of all available protos in the current project
  foreach (const QString &protoName, WbProtoManager::instance()->nameList(WbProtoManager::PROTO_PROJECT)) {
    if (protoName.contains(regexp))
      projectProtosItem->addChild(new QTreeWidgetItem(projectProtosItem, QStringList(protoName)));
  }
  // list of all available protos in the current project
  foreach (const QString &protoName, WbProtoManager::instance()->nameList(WbProtoManager::PROTO_EXTRA)) {
    if (protoName.contains(regexp))
      extraProtosItem->addChild(new QTreeWidgetItem(extraProtosItem, QStringList(protoName)));
  }
  // list of all available protos among the webots ones
  foreach (const QString &protoName, WbProtoManager::instance()->nameList(WbProtoManager::PROTO_WEBOTS)) {
    if (protoName.contains(regexp))
      webotsProtosItem->addChild(new QTreeWidgetItem(webotsProtosItem, QStringList(protoName)));
  }

  if (nodesItem->childCount() > 0)
    mTree->addTopLevelItem(nodesItem);
  if (worldProtosItem->childCount() > 0)
    mTree->addTopLevelItem(worldProtosItem);
  if (projectProtosItem->childCount() > 0)
    mTree->addTopLevelItem(projectProtosItem);
  if (extraProtosItem->childCount() > 0)
    mTree->addTopLevelItem(extraProtosItem);
  if (webotsProtosItem->childCount() > 0)
    mTree->addTopLevelItem(webotsProtosItem);

  if (mFindLineEdit->text().length() > 0)
    mTree->expandAll();
}

void WbNewProtoWizard::updateBaseNode() {
  qDeleteAll(mFields->children());
  mExposedFieldCheckBoxes.clear();

  if (mTree->selectedItems().size() == 0)
    return;

  const QTreeWidgetItem *const selectedItem = mTree->selectedItems().at(0);
  const QTreeWidgetItem *topLevel = selectedItem;
  while (topLevel->parent())
    topLevel = topLevel->parent();

  if (selectedItem->childCount() > 0 || topLevel == selectedItem) {
    mBaseNode = "";  // selected a folder
    return;
  } else
    mBaseNode = selectedItem->text(0);

  QStringList fieldNames;
  if (topLevel->type() == BASE_NODE) {
    WbNodeModel *nodeModel = WbNodeModel::findModel(mBaseNode);
    fieldNames = nodeModel->fieldNames();
    mIsProtoNode = false;
  } else {
    WbProtoModel *protoModel =
      WbProtoManager::instance()->customFindModel(mBaseNode, WbStandardPaths::projectsPath());  // TODO: still functional?
    if (protoModel) {
      fieldNames = protoModel->parameterNames();
      mIsProtoNode = true;
    } else
      WbLog::error(tr("Impossible to instantiate '%1' PROTO node.").arg(mBaseNode));
  }

  QScrollArea *scrollArea = new QScrollArea();
  scrollArea->setHorizontalScrollBarPolicy(Qt::ScrollBarAsNeeded);
  scrollArea->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOn);
  scrollArea->setWidgetResizable(true);

  QWidget *mainWidget = new QWidget();
  QVBoxLayout *fieldsLayout = new QVBoxLayout(mFields);
  QVBoxLayout *layout = new QVBoxLayout(mainWidget);

  if (fieldNames.size() > 0) {
    QCheckBox *selectAll = new QCheckBox();
    selectAll->setText("select all");
    mExposedFieldCheckBoxes.push_back(selectAll);
    layout->addWidget(selectAll);
    connect(selectAll, SIGNAL(stateChanged(int)), this, SLOT(updateCheckBox(int)));

    foreach (const QString &name, fieldNames) {
      mExposedFieldCheckBoxes.push_back(new QCheckBox(name));
      if (defaultFields.contains(name))
        mExposedFieldCheckBoxes.back()->setChecked(true);
      layout->addWidget(mExposedFieldCheckBoxes.back());
    }
  } else
    layout->addWidget(new QLabel("No fields."));

  scrollArea->setWidget(mainWidget);
  fieldsLayout->addWidget(scrollArea);
}

void WbNewProtoWizard::updateCheckBox(int state) {
  // if the "expose all" checkbox is checked, activate all others
  for (int i = 1; i < mExposedFieldCheckBoxes.size(); ++i)
    mExposedFieldCheckBoxes[i]->setChecked(state);
}

QWizardPage *WbNewProtoWizard::createConclusionPage() {
  QWizardPage *page = new QWizardPage(this);

  page->setTitle(tr("Conclusion"));
  page->setSubTitle(tr("The following directory and files will be generated."));

  // add files path to be created
  mFilesLabel = new QLabel(page);
  QVBoxLayout *layout = new QVBoxLayout(page);
  layout->addWidget(mFilesLabel);

  // add space
  layout->addSpacing(30);

  // add check box
  mEditCheckBox = new QCheckBox(page);
  mEditCheckBox->setChecked(true);
  layout->addWidget(mEditCheckBox);

  return page;
}
