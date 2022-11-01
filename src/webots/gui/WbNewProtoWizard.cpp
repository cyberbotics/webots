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
#include "WbParser.hpp"
#include "WbPreferences.hpp"
#include "WbProject.hpp"
#include "WbProtoManager.hpp"
#include "WbProtoModel.hpp"
#include "WbStandardPaths.hpp"
#include "WbTokenizer.hpp"
#include "WbUrl.hpp"
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

static const QStringList defaultFields = {"translation", "rotation", "name", "controller"};

WbNewProtoWizard::WbNewProtoWizard(QWidget *parent) : QWizard(parent) {
  mNeedsEdit = false;
  mIsProtoNode = false;
  mRetrievalTriggered = false;

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
  mProtoFullPath = mProtoDir + mNameEdit->text();
  if (!mProtoFullPath.endsWith(".proto"))
    mProtoFullPath += ".proto";

  // update check box message
  mEditCheckBox->setText(tr("Open '%1.proto' in Text Editor.").arg(mNameEdit->text()));

  QString files = mProtoDir + "\n" + mProtoFullPath;
  mFilesLabel->setText(QDir::toNativeSeparators(files));
}

bool WbNewProtoWizard::validateCurrentPage() {
  updateUI();

  if (currentId() == NAME) {
    if (mNameEdit->text().isEmpty()) {
      WbMessageBox::warning(tr("Please specify a PROTO name."), this, tr("Invalid PROTO name"));
      return false;
    }
    QString path = WbProject::current()->protosPath() + mNameEdit->text();
    if (!path.endsWith(".proto"))
      path += ".proto";
    if (QFile::exists(path)) {
      WbMessageBox::warning(tr("A PROTO file with this name already exists, please choose a different name."), this,
                            tr("Invalid PROTO name"));
      return false;
    }
  }
  return true;
}

void WbNewProtoWizard::accept() {
  // prior to generating the PROTO we need to ensure that the one it derives upon is locally available, therefore we need
  // to download it. The reason is that in the process of generating the new PROTO, we might need to read information from the
  // base one (for example, to know the declaration of the sub-proto it requires)
  if (!mRetrievalTriggered && mIsProtoNode) {
    assert(!mBaseNode.isEmpty());
    connect(WbProtoManager::instance(), &WbProtoManager::retrievalCompleted, this, &WbNewProtoWizard::accept);
    mRetrievalTriggered = true;  // the second time the accept function is called, no retrieval should occur
    WbProtoManager::instance()->retrieveExternProto(WbProtoManager::instance()->protoUrl(mBaseNode, mCategory));
    return;
  }

  if (generateProto())
    WbLog::info(tr("PROTO '%1' added to your project's 'protos' directory.").arg(QFileInfo(mProtoFullPath).fileName()));

  QDialog::accept();
}

bool WbNewProtoWizard::generateProto() {
  // create protos directory
  bool success = QDir::root().mkpath(mProtoDir);

  // copy PROTO from template and rename
  const QString src = WbStandardPaths::templatesPath() + "protos/template.proto";
  success = WbFileUtil::copyAndReplaceString(src, mProtoFullPath, "template", mNameEdit->text()) && success;

  if (!success) {
    WbMessageBox::warning(tr("Some directories or files could not be created."), this, tr("PROTO creation failed"));
    return false;
  }

  QFile file(protoName());
  if (!file.open(QIODevice::ReadWrite)) {
    WbMessageBox::warning(tr("PROTO template not found."), this, tr("PROTO creation failed"));
    return false;
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

  QString externPath;
  QString interface;
  QString body;

  // if a base node was selected, define the exposed parameters and PROTO body accordingly
  if (mBaseNode != "") {
    if (mIsProtoNode) {
      // define header
      QString url = WbProtoManager::instance()->protoUrl(mBaseNode, mCategory);
      externPath = QString("EXTERNPROTO \"%1\"\n").arg(url.replace(WbStandardPaths::webotsHomePath(), "webots://"));
      // define interface
      const WbProtoInfo *const info = WbProtoManager::instance()->protoInfo(mBaseNode, mCategory);
      assert(info);

      const QStringList parameterNames = info->parameterNames();
      const QStringList parameters = info->parameters();
      for (int i = 0; i < parameters.size(); ++i) {
        if (mExposedFieldCheckBoxes[i + 1]->isChecked()) {
          if (parameterNames[i] == "controller" || parameterNames[i] == "window")
            interface += QString("  field SFString %1 \"<generic>\"\n").arg(parameterNames[i]);
          else
            interface += "  " + parameters[i] + "\n";
          // if the field parameter refers to another PROTO, add a declaration for those as well
          WbTokenizer tokenizer;
          tokenizer.tokenizeString(parameters[i]);
          WbParser parser(&tokenizer);
          foreach (const QString &node, parser.protoNodeList()) {
            const QString parentUrl = WbUrl::resolveUrl(url);
            QString nestedUrl = WbProtoManager::instance()->findExternProtoDeclarationInFile(parentUrl, node);
            // replace local URL of nested nodes in distributed remote parent nodes
            const QString prefix = WbUrl::computePrefix(parentUrl);
            if (!prefix.isEmpty()) {
              if (!WbUrl::isWeb(nestedUrl)) {
                if (WbUrl::isLocalUrl(nestedUrl))  // replace the prefix (webots://) based on the parent's prefix
                  nestedUrl.replace("webots://", prefix);
                else  // for relative URL manufacture a remote one based on the parent's path
                  nestedUrl = WbUrl::combinePaths(nestedUrl, parentUrl);
              }
            } else {
              // handle case where the sub-proto is relative and the PROTO of the base type is local (webots://)
              if (WbUrl::isLocalUrl(parentUrl) && (!WbUrl::isWeb(nestedUrl) && QDir::isRelativePath(nestedUrl)))
                nestedUrl = WbUrl::combinePaths(nestedUrl, parentUrl);
            }
            const QString declaration =
              QString("EXTERNPROTO \"%1\"\n").arg(nestedUrl.replace(WbStandardPaths::webotsHomePath(), "webots://"));
            if (!externPath.contains(declaration))
              externPath += declaration;
          }
        }
      }

      // if the interface refers to a relative url, turn it into a web one
      QRegularExpressionMatchIterator it = WbUrl::vrmlResourceRegex().globalMatch(interface);
      while (it.hasNext()) {
        const QRegularExpressionMatch match = it.next();
        if (match.hasMatch()) {
          QString asset = match.captured(0);
          asset.replace("\"", "");
          if (!WbUrl::isWeb(asset) && QDir::isRelativePath(asset)) {
            QString newUrl = QString("\"%1\"").arg(WbUrl::combinePaths(asset, info->url()));
            interface.replace(QString("\"%1\"").arg(asset), newUrl.replace(WbStandardPaths::webotsHomePath(), "webots://"));
          }
        }
      }
      // define IS connections in the body
      body += "  " + mBaseNode + " {\n";
      for (int i = 0; i < parameterNames.size(); ++i)
        if (mExposedFieldCheckBoxes[i + 1]->isChecked())
          body += "    " + parameterNames[i] + " IS " + parameterNames[i] + "\n";
      body += "  }";
    } else {
      // define interface
      const WbNodeModel *const nodeModel = WbNodeModel::findModel(mBaseNode);
      const QList<WbFieldModel *> fieldModels = nodeModel->fieldModels();
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
          interface += "  field " + defaultValue->vrmlTypeName() + " " + fieldModels[i]->name() + " " + vrmlDefaultValue + "\n";
        }
      }
      interface.chop(1);  // chop new line
      // define IS connections in the body
      body += "  " + mBaseNode + " {\n";
      for (int i = 0; i < fieldModels.size(); ++i)
        if (mExposedFieldCheckBoxes[i + 1]->isChecked())
          body += "    " + fieldModels[i]->name() + " IS " + fieldModels[i]->name() + "\n";
      body += "  }";
    }
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
  protoContent.replace(QByteArray("%externproto%"), externPath.toUtf8());
  protoContent.replace(QByteArray("%name%"), mNameEdit->text().toUtf8());
  protoContent.replace(QByteArray("%release%"), release.toUtf8());
  protoContent.replace(QByteArray("%body%"), body.toUtf8());
  protoContent.replace(QByteArray("%interface%"), interface.toUtf8());

  file.seek(0);
  file.write(protoContent);
  file.resize(file.pos());
  file.close();

  mNeedsEdit = mEditCheckBox->isChecked();

  return true;
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

  QTreeWidgetItem *const nodesItem = new QTreeWidgetItem(QStringList(tr("Base nodes")), WbProtoManager::BASE_NODE);
  QTreeWidgetItem *const worldProtosItem =
    new QTreeWidgetItem(QStringList(tr("PROTO nodes (Current World File)")), WbProtoManager::PROTO_WORLD);
  QTreeWidgetItem *const projectProtosItem =
    new QTreeWidgetItem(QStringList(tr("PROTO nodes (Current Project)")), WbProtoManager::PROTO_PROJECT);
  QTreeWidgetItem *const extraProtosItem =
    new QTreeWidgetItem(QStringList(tr("PROTO nodes (Extra Projects)")), WbProtoManager::PROTO_EXTRA);
  QTreeWidgetItem *const webotsProtosItem =
    new QTreeWidgetItem(QStringList(tr("PROTO nodes (Webots Projects)")), WbProtoManager::PROTO_WEBOTS);

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

  const int categories[4] = {WbProtoManager::PROTO_WORLD, WbProtoManager::PROTO_PROJECT, WbProtoManager::PROTO_EXTRA,
                             WbProtoManager::PROTO_WEBOTS};
  QTreeWidgetItem *const items[4] = {worldProtosItem, projectProtosItem, extraProtosItem, webotsProtosItem};
  for (int i = 0; i < 4; ++i) {
    WbProtoManager::instance()->generateProtoInfoMap(categories[i]);
    QMapIterator<QString, WbProtoInfo *> it(WbProtoManager::instance()->protoInfoMap(categories[i]));
    while (it.hasNext()) {
      const QString &protoName = it.next().key();
      if (protoName.contains(regexp) && !it.value()->tags().contains("hidden") && !it.value()->tags().contains("deprecated"))
        items[i]->addChild(new QTreeWidgetItem(items[i], QStringList(protoName)));
    }
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
  mCategory = topLevel->type();
  if (mCategory == WbProtoManager::BASE_NODE) {
    WbNodeModel *nodeModel = WbNodeModel::findModel(mBaseNode);
    fieldNames = nodeModel->fieldNames();
    mIsProtoNode = false;
  } else {
    const WbProtoInfo *info = WbProtoManager::instance()->protoInfo(mBaseNode, mCategory);
    if (info) {
      fieldNames = info->parameterNames();
      mIsProtoNode = true;
    }
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
    selectAll->setText(tr("select all"));
    mExposedFieldCheckBoxes.push_back(selectAll);
    layout->addWidget(selectAll);
    connect(selectAll, &QCheckBox::stateChanged, this, &WbNewProtoWizard::updateCheckBox);

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
