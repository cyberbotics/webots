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
#include "WbMessageBox.hpp"
#include "WbNodeModel.hpp"
#include "WbPreferences.hpp"
#include "WbProject.hpp"
#include "WbProtoList.hpp"
#include "WbProtoModel.hpp"
#include "WbStandardPaths.hpp"
#include "WbVersion.hpp"

#include <QtCore/QDirIterator>

#include <QtGui/QRegExpValidator>

#include <QtWidgets/QCheckBox>
#include <QtWidgets/QLabel>
#include <QtWidgets/QScrollArea>
#include <QtWidgets/QSizePolicy>
#include <QtWidgets/QTreeWidgetItem>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWizardPage>

enum { INTRO, LANGUAGE, NAME, CONCLUSION };  // TODO: modify accordingly

enum { BASE_NODE_LIST = 10001, PROTO_NODE_LIST = 10002 };

WbNewProtoWizard::WbNewProtoWizard(QWidget *parent) : QWizard(parent) {
  mNeedsEdit = false;
  /*
  // prepare list of PROTO files
  QDirIterator it(WbStandardPaths::projectsPath(), QDirIterator::Subdirectories);
  while (it.hasNext()) {
    QFileInfo file(it.next());
    if (!file.isDir() && file.fileName().endsWith(".proto", Qt::CaseInsensitive))
      mProtoFiles.insert(file.baseName(), file.filePath());
  }
  */

  addPage(createIntroPage());
  addPage(createNamePage());
  addPage(createTagsPage());
  addPage(createBaseNodeSelectorPage());
  // addPage(createExposedFieldSelectorPage());
  addPage(createConclusionPage());

  mProceduralCheckBox->setChecked(false);
  mProceduralCheckBox->setText(tr("Procedural PROTO"));
  mProceduralCheckBox->setToolTip(
    tr("By enabling this option, JavaScript template scripting can be used to generate PROTO in a procedural way."));
  mNonDeterministic->setChecked(false);
  mNonDeterministic->setText(tr("Non-deterministic PROTO"));
  mNonDeterministic->setToolTip(tr("A non-deterministic PROTO is a PROTO where the same fields can potentially yield a "
                                   "different result from run to run. This "
                                   "is often the case if random number generation with time-based seeds are employed."));
  mHiddenCheckBox->setChecked(false);
  mHiddenCheckBox->setText(tr("Hidden PROTO"));
  mHiddenCheckBox->setToolTip(
    tr("A hidden PROTO will not appear in the list when adding a new node. This tag is often used for "
       "sub-PROTO, as uninteresting components of a larger node."));

  setOption(QWizard::NoCancelButton, false);
  setOption(QWizard::CancelButtonOnLeft, true);
  setWindowTitle(tr("Create a new PROTO"));

  // setMaximumSize(800, 500);
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
  QString src = WbStandardPaths::templatesPath() + "protos/template.proto";
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
      tags = "# template language: javascript\n# tags: ";
    else
      tags = "# tags: ";

    if (mNonDeterministic->isChecked())
      tags += "nonDeterministic, ";
    if (mHiddenCheckBox->isChecked())
      tags += "hidden, ";

    if (mNonDeterministic->isChecked() || mHiddenCheckBox->isChecked())
      tags.chop(2);
    else
      tags.chop(QString("# tags: ").length());

    QList<WbFieldModel *> fieldModels;
    if (mType == BASE_NODE) {
      WbNodeModel *nodeModel = WbNodeModel::findModel(mBaseNode);
      fieldModels = nodeModel->fieldModels();
    } else {
      WbProtoModel *protoModel = WbProtoList::current()->findModel(mBaseNode, "");
      assert(protoModel);
      fieldModels = protoModel->fieldModels();
      printf("proto model null ? %d\n", protoModel == NULL);
    }
    assert(mExposedFieldCheckBoxes.size() - 1 == fieldModels.size());  // extra entry is "expose all" checkbox

    QString parameters = "";
    for (int i = 0; i < fieldModels.size(); ++i) {
      if (mExposedFieldCheckBoxes[i + 1]->isChecked()) {
        // foreach (WbFieldModel *fieldModel, fieldModels) {
        WbValue *defaultValue = fieldModels[i]->defaultValue();
        parameters +=
          "  field " + defaultValue->vrmlTypeName() + " " + fieldModels[i]->name() + " " + defaultValue->toString() + "\n";
        // printf("%s", parameters.toUtf8().constData());
      }
    }

    parameters.chop(1);  // chop new line

    QString version = WbApplicationInfo::version().toString(false);
    // QString body = mBaseNode.isEmpty() ? "" : "  " + mBaseNode + " {\n  }";
    QString body = "  " + mBaseNode + " {\n";
    // foreach (WbFieldModel *fieldModel, fieldModels) {
    for (int i = 0; i < fieldModels.size(); ++i) {
      if (mExposedFieldCheckBoxes[i + 1]->isChecked())
        body += "    " + fieldModels[i]->name() + " IS " + fieldModels[i]->name() + "\n";
    }
    body += "  }";

    protoContent.replace(QByteArray("%tags%"), tags.toUtf8());
    protoContent.replace(QByteArray("%name%"), mNameEdit->text().toUtf8());
    protoContent.replace(QByteArray("%release%"), version.toUtf8());
    protoContent.replace(QByteArray("%basenode%"), body.toUtf8());
    protoContent.replace(QByteArray("%parameters%"), parameters.toUtf8());

    file.seek(0);
    file.write(protoContent);
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

  QLabel *label = new QLabel(tr("This wizard will help you creating a new PROTO."), page);

  QVBoxLayout *layout = new QVBoxLayout(page);
  layout->addWidget(label);

  return page;
}

QWizardPage *WbNewProtoWizard::createNamePage() {
  QWizardPage *page = new QWizardPage(this);

  page->setTitle(tr("Name selection"));
  page->setSubTitle(tr("Please choose a name for your PROTO node."));

  QLabel *nameLabel = new QLabel(tr("PROTO name:"), page);
  mNameEdit = new WbLineEdit("my_proto", page);
  mNameEdit->setValidator(new QRegExpValidator(QRegExp("[a-zA-Z0-9_-]*"), page));
  nameLabel->setBuddy(mNameEdit);

  QHBoxLayout *layout = new QHBoxLayout(page);
  layout->addWidget(nameLabel);
  layout->addWidget(mNameEdit);

  return page;
}

QWizardPage *WbNewProtoWizard::createTagsPage() {
  QWizardPage *page = new QWizardPage(this);

  page->setTitle(tr("Tags selection"));
  page->setSubTitle(tr("Please choose the tags of your PROTO."));

  mHiddenCheckBox = new QCheckBox(page);
  mNonDeterministic = new QCheckBox(page);
  mProceduralCheckBox = new QCheckBox(page);
  QVBoxLayout *layout = new QVBoxLayout(page);
  layout->addWidget(mProceduralCheckBox);
  layout->addWidget(mNonDeterministic);
  layout->addWidget(mHiddenCheckBox);

  return page;
}

QWizardPage *WbNewProtoWizard::createBaseNodeSelectorPage() {
  QWizardPage *page = new QWizardPage(this);

  page->setTitle(tr("Base node selection"));
  page->setSubTitle(tr("Please choose the base node from which the PROTO will inherit."));

  QHBoxLayout *const mainLayout = new QHBoxLayout(page);
  QVBoxLayout *const nodeListLayout = new QVBoxLayout();
  QVBoxLayout *const fieldListLayout = new QVBoxLayout();

  QFont font;
  font.fromString(WbPreferences::instance()->value("Editor/font").toString());
  mFindLineEdit = new QLineEdit(this);
  mFindLineEdit->setFont(font);
  mFindLineEdit->setClearButtonEnabled(true);

  // QLabel *info = new QLabel();
  // info->setText(tr("Select a node"));

  mTree = new QTreeWidget();

  // fieldListLayout->addWidget(info);
  mFields = new QWidget(this);
  fieldListLayout->addWidget(mFields);

  QSizePolicy policy(QSizePolicy::Preferred, QSizePolicy::Preferred);
  policy.setHorizontalStretch(1);

  mFindLineEdit->setSizePolicy(policy);
  mTree->setSizePolicy(policy);
  // info->setSizePolicy(policy);
  mFields->setSizePolicy(policy);

  nodeListLayout->addWidget(mFindLineEdit);
  nodeListLayout->addWidget(mTree);

  mainLayout->addLayout(nodeListLayout);
  mainLayout->addLayout(fieldListLayout);

  connect(mFindLineEdit, &QLineEdit::textChanged, this, &WbNewProtoWizard::updateNodeTree);
  connect(mTree, &QTreeWidget::itemSelectionChanged, this, &WbNewProtoWizard::updateBaseNode);
  updateNodeTree();

  return page;
}

void WbNewProtoWizard::updateNodeTree() {
  printf("update node\n");
  mTree->clear();
  mTree->setHeaderHidden(true);

  QTreeWidgetItem *const nodesItem = new QTreeWidgetItem(QStringList(tr("Base nodes")), BASE_NODE_LIST);
  QStringList nodes = WbNodeModel::baseModelNames();
  foreach (const QString &basicNodeName, nodes) {
    QFileInfo fileInfo(basicNodeName);
    if (fileInfo.baseName().contains(QRegExp(mFindLineEdit->text(), Qt::CaseInsensitive, QRegExp::Wildcard))) {
      QTreeWidgetItem *item = new QTreeWidgetItem(nodesItem, QStringList(fileInfo.baseName()));
      nodesItem->addChild(item);
    }
  }

  QTreeWidgetItem *const protosItem = new QTreeWidgetItem(QStringList(tr("PROTO nodes")), PROTO_NODE_LIST);
  QStringList protoNodesNames = WbProtoList::current()->fileList(WbProtoList::PROJECTS_PROTO_CACHE);
  foreach (const QString &protoName, protoNodesNames) {
    // QMapIterator<QString, QString> it(mProtoFiles);
    // while (it.hasNext()) {
    //  it.next();
    if (protoName.contains(QRegExp(mFindLineEdit->text(), Qt::CaseInsensitive, QRegExp::Wildcard))) {
      QTreeWidgetItem *item = new QTreeWidgetItem(protosItem, QStringList(protoName));
      protosItem->addChild(item);
    }
  }

  mTree->addTopLevelItem(nodesItem);
  mTree->addTopLevelItem(protosItem);

  if (mFindLineEdit->text().length() > 0)
    mTree->expandAll();
}

void WbNewProtoWizard::updateBaseNode() {
  printf("update base\n");
  qDeleteAll(mFields->children());
  mExposedFieldCheckBoxes.clear();

  const QTreeWidgetItem *const selectedItem = mTree->selectedItems().at(0);
  if (!selectedItem)
    return;

  const QTreeWidgetItem *topLevel = selectedItem;
  while (topLevel->parent())
    topLevel = topLevel->parent();

  if (selectedItem->childCount() > 0 || topLevel == selectedItem) {
    mBaseNode = "";  // selected a folder
    return;
  } else
    mBaseNode = selectedItem->text(0);

  QStringList fieldNames;
  if (topLevel->type() == PROTO_NODE_LIST) {
    /*
    assert(mProtoFiles[mBaseNode]);
    WbProtoCachedInfo *protoCachedInfo = new WbProtoCachedInfo(mProtoFiles[mBaseNode]);
    bool success = protoCachedInfo->load();
    if (!success || protoCachedInfo->isOutOfDate()) {
      delete protoCachedInfo;
      protoCachedInfo = WbProtoCachedInfo::computeInfo(mBaseNode);
    }
    fieldNames = protoCachedInfo->parameterNames();
    */
    WbProtoModel *protoModel = WbProtoList::current()->findModel(mBaseNode, WbStandardPaths::projectsPath());
    fieldNames = protoModel->parameterNames();
    mType = PROTO_NODE;
  } else {
    WbNodeModel *nodeModel = WbNodeModel::findModel(mBaseNode);
    fieldNames = nodeModel->fieldNames();
    mType = BASE_NODE;
  }

  printf("# models: %d\n", fieldNames.size());

  QSizePolicy policy(QSizePolicy::Preferred, QSizePolicy::Preferred);
  policy.setHorizontalStretch(1);

  QScrollArea *scrollArea = new QScrollArea();
  scrollArea->viewport()->setBackgroundRole(QPalette::Dark);
  scrollArea->viewport()->setAutoFillBackground(true);
  scrollArea->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  scrollArea->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOn);

  QWidget *mainWidget = new QWidget();
  QVBoxLayout *fieldsLayout = new QVBoxLayout(mFields);
  QVBoxLayout *layout = new QVBoxLayout(mainWidget);

  QCheckBox *exposeAll = new QCheckBox();
  exposeAll->setText("expose all");
  // allCheckBox->setChecked(true);
  mExposedFieldCheckBoxes.push_back(exposeAll);
  connect(exposeAll, SIGNAL(stateChanged(int)), this, SLOT(updateCheckBox(int)));
  layout->addWidget(exposeAll);

  foreach (const QString &name, fieldNames) {
    // printf("%s\n", protoParameter.toUtf8().constData());
    // QCheckBox *fieldCheckBox = new QCheckBox();
    // fieldCheckBox->setText(fieldModel->name());
    mExposedFieldCheckBoxes.push_back(new QCheckBox(name));
    layout->addWidget(mExposedFieldCheckBoxes.back());
  }

  scrollArea->setWidget(mainWidget);
  fieldsLayout->addWidget(scrollArea);
}

void WbNewProtoWizard::updateCheckBox(int state) {
  for (int i = 1; i < mExposedFieldCheckBoxes.size(); ++i)
    mExposedFieldCheckBoxes[i]->setChecked(state);
}

QWizardPage *WbNewProtoWizard::createExposedFieldSelectorPage() {
  /*
    QWizardPage *page = new QWizardPage(this);

    page->setTitle(tr("Exposed field selection"));
    page->setSubTitle(tr("Please choose which fields of the %1 node should be modifiable from the scene
    tree.").arg(mBaseNode));

    mBaseNode = QString("Accelerometer");
    WbNodeModel *nodeModel = WbNodeModel::findModel(mBaseNode);

    // QVBoxLayout *layout = new QVBoxLayout(page);

    printf("null field model? %d\n", nodeModel == NULL);
    const QList<WbFieldModel *> &fieldModels = nodeModel->fieldModels();

    int n = fieldModels.size();
    // printf("fields: %d\n", n);

    // QCheckBox *fieldCheckBoxes[n];

    QScrollArea *scrollArea = new QScrollArea(page);
    // scrollArea->setGeometry(0, 0, 800, 500);
    QWidget *mainWidget = new QWidget();

    QVBoxLayout *layout = new QVBoxLayout(mainWidget);

    //QCheckBox *allCheckBox = new QCheckBox();
    //allCheckBox->setText("all");
    //allCheckBox->setChecked(true);
    //layout->addWidget(allCheckBox);

    foreach (WbFieldModel *fieldModel, fieldModels) {
      // printf("%s\n", protoParameter.toUtf8().constData());
      QCheckBox *fieldCheckBox = new QCheckBox();
      fieldCheckBox->setText(fieldModel->name());
      fieldCheckBox->setEnabled(false);
      layout->addWidget(fieldCheckBox);
    }

    scrollArea->setWidget(mainWidget);

    return page;
    */
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
