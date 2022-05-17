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

#include "WbAddNodeDialog.hpp"

#include <WbDownloader.hpp>
#include <WbNetwork.hpp>
#include "WbBaseNode.hpp"
#include "WbDesktopServices.hpp"
#include "WbDictionary.hpp"
#include "WbField.hpp"
#include "WbLog.hpp"
#include "WbMFNode.hpp"
#include "WbNode.hpp"
#include "WbNodeModel.hpp"
#include "WbNodeUtilities.hpp"
#include "WbPreferences.hpp"
#include "WbProject.hpp"
#include "WbProtoList.hpp"
#include "WbProtoModel.hpp"
#include "WbSFNode.hpp"
#include "WbSimulationState.hpp"
#include "WbStandardPaths.hpp"
#include "WbUrl.hpp"

#include <QtCore/QRegularExpression>
#include <QtWidgets/QDialogButtonBox>
#include <QtWidgets/QFileDialog>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QPlainTextEdit>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QTreeWidget>
#include <QtWidgets/QTreeWidgetItem>

#include <cassert>

enum { NEW = 10001, PROTO_WORLD = 10002, PROTO_PROJECT = 10003, PROTO_EXTRA = 10004, PROTO_WEBOTS = 10005, USE = 10006 };

WbAddNodeDialog::WbAddNodeDialog(WbNode *currentNode, WbField *field, int index, QWidget *parent) :
  QDialog(parent),
  mCurrentNode(currentNode),
  mField(field),
  mIndex(index),
  mUsesItem(NULL),
  mNewNodeType(UNKNOWN),
  mDefNodeIndex(-1),
  mActionType(CREATE),
  mIsFolderItemSelected(true),
  mDownloader(NULL) {
  assert(mCurrentNode && mField);

  // check if top node is a robot node
  const WbNode *const topNode =
    field ? WbNodeUtilities::findTopNode(mCurrentNode) : WbNodeUtilities::findTopNode(mCurrentNode->parentNode());
  mHasRobotTopNode = topNode ? WbNodeUtilities::isRobotTypeName(topNode->nodeModelName()) : false;

  setWindowTitle(tr("Add a node"));

  mTree = new QTreeWidget(this);
  mTree->setHeaderHidden(true);
  mTree->setSelectionMode(QAbstractItemView::SingleSelection);
  mTree->setSortingEnabled(true);
  mTree->sortByColumn(0, Qt::AscendingOrder);
  connect(mTree, &QTreeWidget::doubleClicked, this, &WbAddNodeDialog::checkAndAddSelectedItem);

  QFont font;
  font.fromString(WbPreferences::instance()->value("Editor/font").toString());

  mFindLineEdit = new QLineEdit(this);
  mFindLineEdit->setFont(font);
  mFindLineEdit->setClearButtonEnabled(true);
  connect(mFindLineEdit, &QLineEdit::textChanged, this, &WbAddNodeDialog::buildTree);

  QHBoxLayout *filterLayout = new QHBoxLayout;
  filterLayout->addStretch();
  QLabel *findLabel = new QLabel(tr("Find:"), this);
  filterLayout->addWidget(findLabel);
  filterLayout->addWidget(mFindLineEdit);

  QString toolTip(tr("Filter node names. "
                     "Only the node names containing the given string are displayed in the tree below. "
                     "Regular expressions can be used."));
  findLabel->setToolTip(toolTip);
  mFindLineEdit->setToolTip(toolTip);

  mInfoText = new QPlainTextEdit(this);
  mInfoText->setFont(font);
  mInfoText->setReadOnly(true);
  mInfoText->setFocusPolicy(Qt::ClickFocus);

  mNodeInfoGroupBox = new QGroupBox(this);
  mNodeInfoGroupBox->setObjectName("dialogInfoGroupBox");
  mNodeInfoGroupBox->setFont(font);
  mNodeInfoGroupBox->setFlat(false);

  mPixmapLabel = new QLabel(this);
  mPixmapLabel->setObjectName("nodePixmapLabel");
  mPixmapLabel->setMinimumSize(128, 128);
  mPixmapLabel->setMaximumSize(mPixmapLabel->minimumSize());

  mDocumentationLabel = new QLabel(this);
  mDocumentationLabel->setObjectName("documentationLabel");
  connect(mDocumentationLabel, &QLabel::linkActivated, &WbDesktopServices::openUrl);
  mDocumentationLabel->setWordWrap(true);
  font.setItalic(true);
  mDocumentationLabel->setFont(font);

  mLicenseLabel = new QLabel(this);
  mLicenseLabel->setObjectName("licenseLabel");
  connect(mLicenseLabel, &QLabel::linkActivated, &WbDesktopServices::openUrl);
  mLicenseLabel->setWordWrap(true);
  font.setItalic(true);
  mLicenseLabel->setFont(font);

  QPushButton *const cancelButton = new QPushButton(tr("Cancel"), this);
  cancelButton->setFocusPolicy(Qt::ClickFocus);
  mAddButton = new QPushButton(tr("Add"), this);
  mAddButton->setFocusPolicy(Qt::ClickFocus);
  connect(cancelButton, &QPushButton::pressed, this, &WbAddNodeDialog::reject);
  connect(mAddButton, &QPushButton::pressed, this, &WbAddNodeDialog::accept);

  QHBoxLayout *const mainLayout = new QHBoxLayout(this);
  QVBoxLayout *const rightPaneLayout = new QVBoxLayout();
  QVBoxLayout *const nodeInfoLayout = new QVBoxLayout();

  nodeInfoLayout->addWidget(mPixmapLabel, 0, Qt::AlignHCenter);
  nodeInfoLayout->addWidget(mInfoText);
  nodeInfoLayout->addWidget(mDocumentationLabel);
  nodeInfoLayout->addWidget(mLicenseLabel);
  mNodeInfoGroupBox->setLayout(nodeInfoLayout);

  QDialogButtonBox *const buttonBox = new QDialogButtonBox(this);
  buttonBox->addButton(mAddButton, QDialogButtonBox::AcceptRole);

  QPushButton *const importButton = new QPushButton(tr("Import..."), this);
  importButton->setEnabled(mField->isMultiple());
  importButton->setFocusPolicy(Qt::ClickFocus);
  connect(importButton, &QPushButton::pressed, this, &WbAddNodeDialog::import);
  buttonBox->addButton(importButton, QDialogButtonBox::AcceptRole);
  buttonBox->addButton(cancelButton, QDialogButtonBox::RejectRole);
  buttonBox->setFocusPolicy(Qt::ClickFocus);

  QHBoxLayout *buttonLayout = new QHBoxLayout();
  buttonLayout->addWidget(buttonBox);

  if (WbSimulationState::instance()->hasStarted()) {
    QPixmap pixmap("coreIcons:warning.png");
    QLabel *pixmapLabel = new QLabel(this);
    pixmapLabel->setPixmap(pixmap.scaledToHeight(20, Qt::SmoothTransformation));
    pixmapLabel->setToolTip(tr("The simulation has run!"));
    buttonLayout->addWidget(pixmapLabel);
  }

  rightPaneLayout->addLayout(filterLayout);
  rightPaneLayout->addWidget(mNodeInfoGroupBox);
  rightPaneLayout->addLayout(buttonLayout);

  mainLayout->addWidget(mTree);
  mainLayout->addLayout(rightPaneLayout);

  // populate the tree with suitable nodes
  buildTree();

  setMinimumSize(800, 500);

  connect(mTree, &QTreeWidget::itemSelectionChanged, this, &WbAddNodeDialog::updateItemInfo);
}

WbAddNodeDialog::~WbAddNodeDialog() {
}

void WbAddNodeDialog::downloadIcon(const QString &url) {
  if (mDownloader != NULL && mDownloader->hasFinished())
    delete mDownloader;

  mDownloader = new WbDownloader(this);
  connect(mDownloader, &WbDownloader::complete, this, &WbAddNodeDialog::downloadUpdate);

  mDownloader->download(QUrl(url));
}

void WbAddNodeDialog::downloadUpdate() {
  if (mDownloader && !mDownloader->error().isEmpty()) {
    WbLog::error(mDownloader->error());  // failure downloading or file does not exist (404)
    delete mDownloader;
    mDownloader = NULL;
    return;
  }

  QString pixmapPath = WbNetwork::instance()->get(mDownloader->url().toString());
  QPixmap pixmap(pixmapPath);
  if (!pixmap.isNull()) {
    if (pixmap.size() != QSize(128, 128)) {
      WbLog::warning(tr("The \"%1\" icon should have a dimension of 128x128 pixels.").arg(pixmapPath));
      pixmap = pixmap.scaled(128, 128);
    }
    mPixmapLabel->show();
    mPixmapLabel->setPixmap(pixmap);
  }
}

QString WbAddNodeDialog::modelName() const {
  QString modelName(mTree->selectedItems().at(0)->text(MODEL_NAME));
  if (mNewNodeType == PROTO || mNewNodeType == USE) {
    // return only proto/use name without model name
    return modelName.split(QRegularExpression("\\W+"))[0];
  }
  return modelName;
}

QString WbAddNodeDialog::protoFilePath() const {
  if (mNewNodeType != PROTO)
    return QString();

  return mTree->selectedItems().at(0)->text(FILE_NAME);
}

WbNode *WbAddNodeDialog::defNode() const {
  assert(mDefNodeIndex >= 0);
  return mDefNodes[mDefNodeIndex];
}

void WbAddNodeDialog::updateItemInfo() {
  if (mTree->selectedItems().size() != 1)
    return;

  const QTreeWidgetItem *const selectedItem = mTree->selectedItems().at(0);
  const QTreeWidgetItem *topLevel = selectedItem;
  while (topLevel->parent())
    topLevel = topLevel->parent();

  const QString selectedNode(selectedItem->text(MODEL_NAME));
  mNodeInfoGroupBox->setTitle(selectedNode);
  bool validForInsertionInBoundingObject = true;
  mLicenseLabel->hide();
  mDocumentationLabel->hide();
  if (selectedItem->childCount() > 0 || topLevel == selectedItem) {
    // a folder is selected
    mIsFolderItemSelected = true;
    mPixmapLabel->hide();

    switch (topLevel->type()) {
      case NEW:
        mInfoText->setPlainText(tr("This folder lists all Webots base nodes that are suitable to insert at (or below) the "
                                   "currently selected Scene Tree line."));
        break;
      case USE:
        mInfoText->setPlainText(
          tr("This folder lists all suitable node that were defined (using DEF) above the current line of the Scene Tree."));
        break;
      case PROTO_WORLD: {
        const QString worldFile = "ASD";  // TODO
        mInfoText->setPlainText(tr("This folder lists all suitable PROTO nodes from the world file: '%1'.").arg(worldFile));
        break;
      }
      case PROTO_PROJECT:
        mInfoText->setPlainText(tr("This folder lists all suitable PROTO nodes from the local 'protos' directory: '%1'.")
                                  .arg(WbProject::current()->protosPath()));
        break;
      case PROTO_EXTRA: {
        QString title("This folder lists all suitable PROTO nodes from the preferences Extra project path and "
                      "the 'WEBOTS_EXTRA_PROJECT_PATH' environment variable:\n");
        foreach (const WbProject *project, *WbProject::extraProjects())
          title.append(QString("- " + project->path() + "\n"));
        mInfoText->setPlainText(title);
      } break;
      case PROTO_WEBOTS:
        mInfoText->setPlainText(tr("This folder lists all suitable PROTO nodes provided by Webots."));
        break;
      default:
        // no information
        mInfoText->setPlainText(tr("No info available."));
        break;
    }

  } else {
    // a node is selected
    mIsFolderItemSelected = false;

    // check if USE node
    switch (topLevel->type()) {
      case USE: {
        mNewNodeType = USE;
        QString boi(selectedItem->text(BOUNDING_OBJECT_INFO));
        validForInsertionInBoundingObject = boi.isEmpty();
        showNodeInfo(selectedItem->text(FILE_NAME), USE, 0, boi);
        mDefNodeIndex = mUsesItem->indexOfChild(const_cast<QTreeWidgetItem *>(selectedItem));
        assert(mDefNodeIndex < mDefNodes.size() && mDefNodeIndex >= 0);
        break;
      }
      case PROTO_WORLD:
      case PROTO_PROJECT:
      case PROTO_EXTRA:
      case PROTO_WEBOTS:
        mDefNodeIndex = -1;
        mNewNodeType = PROTO;
        showNodeInfo(selectedItem->text(FILE_NAME), PROTO, topLevel->type());
        break;
      default:
        mDefNodeIndex = -1;
        mNewNodeType = BASIC;
        showNodeInfo(selectedNode, BASIC, BASIC);
        break;
    }
  }

  mAddButton->setEnabled(!selectedItem->icon(0).isNull() && validForInsertionInBoundingObject);
}

void WbAddNodeDialog::showNodeInfo(const QString &nodeFileName, NodeType nodeType, int variant,
                                   const QString &boundingObjectInfo) {  // TODO: can variant be better?
  QString description;
  QString pixmapPath;

  const QFileInfo fileInfo(nodeFileName);
  const QString modelName = fileInfo.baseName();
  if (nodeType != PROTO && WbNodeModel::isBaseModelName(modelName)) {
    WbNodeModel *nodeModel = WbNodeModel::findModel(modelName);
    assert(nodeModel);
    description = nodeModel->info();

    // set icon path
    pixmapPath = "icons:" + fileInfo.baseName() + ".png";
  } else {
    QMap<QString, WbProtoInfo *> protoList;

    if (variant == PROTO_WORLD) {
      protoList = WbProtoList::instance()->worldProtoList();
      if (!protoList.contains(modelName)) {
        WbLog::error(tr("'%1' is not a known PROTO in this world.\n").arg(modelName));
        return;
      }
    } else if (variant == PROTO_WEBOTS) {
      protoList = WbProtoList::instance()->webotsProtoList();
      if (!protoList.contains(modelName)) {
        WbLog::error(tr("'%1' is not a known Webots PROTO.\n").arg(modelName));
        return;
      }
    }

    WbProtoInfo *info = protoList.value(modelName);

    // set documentation url
    if (!info->documentationUrl().isEmpty()) {
      mDocumentationLabel->show();
      mDocumentationLabel->setText(
        tr("Documentation: <a style='color: #5DADE2;' href='%1'>%1</a>").arg(info->documentationUrl()));
    }

    // set license
    if (!info->license().isEmpty()) {
      QString license =
        info->license() + tr(" <a style='color: #5DADE2;' href='%1'>More information.</a>").arg(info->licenseUrl());
      mLicenseLabel->show();
      mLicenseLabel->setText(tr("License: ") + license);
    }

    mInfoText->clear();

    if (!boundingObjectInfo.isEmpty())
      mInfoText->appendHtml(tr("<font color=\"red\">WARNING: this node contains a Geometry with non-positive dimensions and "
                               "hence cannot be inserted in a bounding object.</font><br/>"));

    description = info->description();
    if (description.isEmpty())
      mInfoText->setPlainText(tr("No info available."));
    else {
      // replace carriage returns with spaces where appropriate:
      // "\n\n" => "\n\n": two consecutive carriage returns are preserved (new paragraph)
      // "\n-"  => "\n-": a carriage return followed by a "-" are preserved (bullet list)
      // "\n"   => " ": a single carriage return is transformed into a space (comment line wrap)
      for (int i = 0; i < description.length(); i++) {
        if (description[i] == '\n') {
          if (i < (description.length() - 1)) {
            if (description[i + 1] == '\n' || description[i + 1] == '-') {
              i++;
              continue;
            }
          }
          description[i] = ' ';
        }
      }
      mInfoText->appendPlainText(description.trimmed());
    }
    mInfoText->moveCursor(QTextCursor::Start);

    pixmapPath = QString("%1icons/%2.png").arg(QUrl(info->url()).adjusted(QUrl::RemoveFilename).toString()).arg(modelName);
    printf("ICON WILL BE AT: %s\n", pixmapPath.toUtf8().constData());
  }

  mPixmapLabel->hide();
  if (!pixmapPath.isEmpty()) {
    if (WbUrl::isWeb(pixmapPath)) {
      if (WbNetwork::instance()->isCached(pixmapPath))
        pixmapPath = WbNetwork::instance()->get(pixmapPath);
      else {
        downloadIcon(pixmapPath);
        return;
      }
    } else if (WbUrl::isLocalUrl(pixmapPath))
      pixmapPath = QDir::cleanPath(WbStandardPaths::webotsHomePath() + pixmapPath.mid(9));

    QPixmap pixmap(pixmapPath);
    if (!pixmap.isNull()) {
      if (pixmap.size() != QSize(128, 128)) {
        WbLog::warning(tr("The \"%1\" icon should have a dimension of 128x128 pixels.").arg(pixmapPath));
        pixmap = pixmap.scaled(128, 128);
      }
      mPixmapLabel->show();
      mPixmapLabel->setPixmap(pixmap);
    }
  }
}

bool WbAddNodeDialog::doFieldRestrictionsAllowNode(const QString &nodeName) const {
  foreach (const WbVariant variant, mField->acceptedValues()) {
    const WbNode *node = variant.toNode();
    assert(node);
    if (node->modelName() == nodeName)
      return true;
  }
  return false;
}

void WbAddNodeDialog::buildTree() {
  mTree->clear();
  mUsesItem = NULL;
  mDefNodes.clear();

  QTreeWidgetItem *const nodesItem = new QTreeWidgetItem(QStringList(tr("Base nodes")), NEW);
  QTreeWidgetItem *const worldProtosItem = new QTreeWidgetItem(QStringList("PROTO nodes (Current World)"), PROTO_WORLD);
  QTreeWidgetItem *const projectProtosItem = new QTreeWidgetItem(QStringList("PROTO nodes (Current Project)"), PROTO_PROJECT);
  QTreeWidgetItem *const extraProtosItem = new QTreeWidgetItem(QStringList(tr("PROTO nodes (Extra Projects)")), PROTO_EXTRA);
  QTreeWidgetItem *const webotsProtosItem = new QTreeWidgetItem(QStringList("PROTO nodes (Webots Projects)"), PROTO_WEBOTS);
  mUsesItem = new QTreeWidgetItem(QStringList("USE"), USE);

  const QStringList basicNodes = WbNodeModel::baseModelNames();
  QTreeWidgetItem *item = NULL;

  const QRegularExpression regexp(
    QRegularExpression::wildcardToRegularExpression(mFindLineEdit->text(), QRegularExpression::UnanchoredWildcardConversion),
    QRegularExpression::CaseInsensitiveOption);

  // add valid basic nodes
  const WbNode::NodeUse nodeUse = static_cast<WbBaseNode *>(mCurrentNode)->nodeUse();
  foreach (const QString &basicNodeName, basicNodes) {
    QFileInfo fileInfo(basicNodeName);
    QString errorMessage;
    if (fileInfo.baseName().contains(regexp) &&
        WbNodeUtilities::isAllowedToInsert(mField, fileInfo.baseName(), mCurrentNode, errorMessage, nodeUse, QString(),
                                           QStringList(fileInfo.baseName()))) {
      item = new QTreeWidgetItem(nodesItem, QStringList(fileInfo.baseName()));
      item->setIcon(0, QIcon("enabledIcons:node.png"));
      nodesItem->addChild(item);
    }
  }

  // add USE nodes that are suitable for insertion
  if (mUsesItem) {
    static const QString INVALID_FOR_INSERTION_IN_BOUNDING_OBJECT("N");

    const WbField *const actualField =
      (mField->isParameter() && !mField->alias().isEmpty()) ? mField->internalFields().at(0) : mField;
    bool boInfo = actualField->name() == "boundingObject";
    if (!boInfo)
      boInfo = nodeUse & WbNode::BOUNDING_OBJECT_USE;

    // populates the DEF-USE dictionary with suitable definitions located above mCurrentNode
    mDefNodes = WbDictionary::instance()->computeDefForInsertion(mCurrentNode, mField, mIndex, true);
    foreach (const WbNode *const defNode, mDefNodes) {
      const QString &currentDefName = defNode->defName();
      const QString &currentModelName = defNode->modelName();
      const QString &currentFullDefName = currentDefName + " (" + currentModelName + ")";
      if (!currentFullDefName.contains(regexp))
        continue;
      if (mField->hasRestrictedValues() &&
          (!doFieldRestrictionsAllowNode(currentModelName) && !doFieldRestrictionsAllowNode(defNode->nodeModelName())))
        continue;
      QString nodeFilePath(currentModelName);
      if (!WbNodeModel::isBaseModelName(currentModelName))
        nodeFilePath = WbProtoList::instance()->findModelPath(currentModelName);

      QStringList strl(QStringList() << currentFullDefName << nodeFilePath);

      if (boInfo && !(dynamic_cast<const WbBaseNode *const>(defNode))->isSuitableForInsertionInBoundingObject())
        strl << INVALID_FOR_INSERTION_IN_BOUNDING_OBJECT;

      QTreeWidgetItem *const child = new QTreeWidgetItem(mUsesItem, strl);

      child->setIcon(0, QIcon("enabledIcons:node.png"));
    }
  }

  // add World PROTO (i.e. referenced as EXTERNPROTO by the world file)
  int nWorldProtosNodes = 0;
  nWorldProtosNodes = addProtosFromProtoList(worldProtosItem, PROTO_WORLD, regexp);
  // add Current Project PROTO (all PROTO locally available in the project location)

  // add Extra PROTO (all PROTO available in the extra location)
  int nExtraProtosNodes = 0;
  nExtraProtosNodes = addProtosFromProtoList(extraProtosItem, PROTO_EXTRA, regexp);
  // add Webots PROTO
  int nWProtosNodes = 0;  // TODO: fix n's
  nWProtosNodes = addProtosFromProtoList(webotsProtosItem, PROTO_WEBOTS, regexp);

  mTree->addTopLevelItem(nodesItem);
  mTree->addTopLevelItem(worldProtosItem);
  mTree->addTopLevelItem(projectProtosItem);
  if (extraProtosItem)
    mTree->addTopLevelItem(extraProtosItem);
  mTree->addTopLevelItem(webotsProtosItem);
  if (mUsesItem)
    mTree->addTopLevelItem(mUsesItem);

  // initial selection
  const int nBasicNodes = nodesItem->childCount();
  const int nUseNodes = mUsesItem ? mUsesItem->childCount() : 0;
  const int nLProtosNodes = 0;  // lprotosItem ? lprotosItem->childCount() : 0;
  const int nAProtosNodes = 0;  // aprotosItem ? aprotosItem->childCount() : 0;

  // if everything can fit in the tree height then show all
  if (nBasicNodes + nUseNodes + nLProtosNodes + nAProtosNodes + nWProtosNodes < 20)
    mTree->expandAll();

  // if no USE nor PROTO items
  if (nBasicNodes && !nUseNodes && !nLProtosNodes && !nAProtosNodes && !nWProtosNodes)
    // then select first basic node
    mTree->setCurrentItem(nodesItem->child(0));
  else
    mTree->setCurrentItem(nodesItem);

  updateItemInfo();
}

int WbAddNodeDialog::addProtosFromProtoList(QTreeWidgetItem *parentItem, int type, const QRegularExpression &regexp) {
  int nAddedNodes = 0;
  const QRegularExpression re("(https://raw.githubusercontent.com/cyberbotics/webots/[a-zA-Z0-9\\-\\_\\+]+/)");
  const WbNode::NodeUse nodeUse = static_cast<WbBaseNode *>(mCurrentNode)->nodeUse();

  bool flattenHierarchy = true;
  QMap<QString, WbProtoInfo *> protoList;
  if (type == PROTO_WORLD) {
    WbProtoList::instance()->generateWorldProtoList();  // TODO: can return the list directly?
    protoList = WbProtoList::instance()->worldProtoList();
  } else if (type == PROTO_EXTRA) {
    WbProtoList::instance()->generateExtraProtoList();
    protoList = WbProtoList::instance()->extraProtoList();
  } else if (type == PROTO_WEBOTS) {
    protoList = WbProtoList::instance()->webotsProtoList();
    flattenHierarchy = false;
  }

  QMapIterator<QString, WbProtoInfo *> it(protoList);
  while (it.hasNext()) {
    it.next();
    WbProtoInfo *info = it.value();

    // don't display PROTOs which contain a "hidden" or a "deprecated" tag
    const QStringList tags = info->tags();
    if (tags.contains("deprecated", Qt::CaseInsensitive) || tags.contains("hidden", Qt::CaseInsensitive))
      continue;

    // don't display PROTO nodes which have been filtered-out by the user's "filter" widget.
    const QString baseType = info->baseType();
    const QString path = info->url().replace("webots://", "").replace(re, "").replace(WbStandardPaths::webotsHomePath(), "");
    if (!path.contains(regexp) && !baseType.contains(regexp))
      continue;

    // don't display non-Robot PROTO nodes containing devices (e.g. Kinect) about to be inserted outside a robot.
    if (!mHasRobotTopNode && !WbNodeUtilities::isRobotTypeName(baseType) && info->needsRobotAncestor())
      continue;

    QString errorMessage;
    const QString nodeName = it.key();
    if (!WbNodeUtilities::isAllowedToInsert(mField, baseType, mCurrentNode, errorMessage, nodeUse, info->slotType(),
                                            QStringList() << baseType << nodeName))
      continue;

    // populate tree
    if (flattenHierarchy) {
      QTreeWidgetItem *item = new QTreeWidgetItem(QStringList() << QString("%1 (%2)").arg(nodeName).arg(baseType) << path);
      parentItem->addChild(item);
      item->setIcon(0, QIcon("enabledIcons:proto.png"));
      ++nAddedNodes;
    } else {
      QStringList categories = path.split('/', Qt::SkipEmptyParts);
      QTreeWidgetItem *parent = parentItem;
      foreach (QString folder, categories) {
        if (folder == "projects" || folder == "protos")
          continue;

        const bool isProto = folder.endsWith(".proto");
        bool exists = false;
        int i;
        for (i = 0; i < parent->childCount(); ++i) {
          if (parent->child(i)->text(0) == folder) {
            exists = true;
            break;
          }
        }

        QTreeWidgetItem *subFolder;
        if (exists)
          subFolder = parent->child(i);
        else {
          const QString name = isProto ? QString("%1 (%2)").arg(nodeName).arg(baseType) : folder;
          subFolder = new QTreeWidgetItem(QStringList() << name << path);
        }

        if (isProto) {
          subFolder->setIcon(0, QIcon("enabledIcons:proto.png"));
          ++nAddedNodes;
        }
        parent->addChild(subFolder);
        parent = subFolder;
      }
    }
  }

  return nAddedNodes;
}

void WbAddNodeDialog::import() {
  static QString lastFolder = QDir::homePath();
  QString fileName = QFileDialog::getOpenFileName(this, tr("Import Webots Object"), lastFolder, tr("Files (*.wbo *.WBO)"));
  if (fileName.isEmpty())
    return;

  lastFolder = QFileInfo(fileName).absolutePath();
  mActionType = IMPORT;
  mImportFileName = fileName;
  accept();
}

void WbAddNodeDialog::checkAndAddSelectedItem() {
  if (mIsFolderItemSelected)
    return;

  accept();
}
