// Copyright 1996-2021 Cyberbotics Ltd.
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

#include "WbDocumentation.hpp"

#include "WbApplicationInfo.hpp"
#include "WbClipboard.hpp"
#include "WbDesktopServices.hpp"
#include "WbStandardPaths.hpp"
#include "WbWebotsUpdateManager.hpp"

#include <QtCore/QUrl>

#include <cassert>

#include <QtGui/QContextMenuEvent>
#include <QtWidgets/QMenu>

#ifdef _WIN32
#include <QtWebKitWidgets/QWebView>
#else
#include <QtWebEngineWidgets/QWebEngineView>
#define QWebView QWebEngineView
#define QWebPage QWebEnginePage
#ifdef __APPLE__
// For the chromium bug on macOS below.
#include <QtWebEngineWidgets/QWebEngineScript>
#include <QtWebEngineWidgets/QWebEngineScriptCollection>
#endif
#endif

#include <iostream>
/*
// Debug code: uncomment to show a web inspector for QtWebKit.
#include <QtWebKitWidgets/QWebInspector>
#include <QtWidgets/QDialog>
#include <QtWidgets/QVBoxLayout>
*/

class DocumentationWebView : public QWebView {
public:
  explicit DocumentationWebView(QWidget *parent = NULL) : QWebView(parent) { setObjectName("DocumentationWebView"); }

protected:
#ifdef _WIN32
  // QWebEngineView::focusInEvent is not working: https://bugreports.qt.io/browse/QTBUG-67852
  // Meanwhile, QWebEnginePage::Copy action is used.
  void focusInEvent(QFocusEvent *event) override {
    // update application actions
    WbActionManager *actionManager = WbActionManager::instance();
    actionManager->enableTextEditActions(false);
    actionManager->setFocusObject(this);
    actionManager->setEnabled(WbAction::COPY, false);
    actionManager->setEnabled(WbAction::CUT, false);
    actionManager->setEnabled(WbAction::PASTE, false);
    actionManager->setEnabled(WbAction::SELECT_ALL, false);
    actionManager->setEnabled(WbAction::UNDO, false);
    actionManager->setEnabled(WbAction::REDO, false);
    actionManager->setEnabled(WbAction::FIND, false);
    actionManager->setEnabled(WbAction::FIND_NEXT, false);
    actionManager->setEnabled(WbAction::FIND_PREVIOUS, false);
  }
#endif

  void contextMenuEvent(QContextMenuEvent *event) override {
    QMenu menu(this);
    menu.addAction(pageAction(QWebPage::Back));
    menu.addAction(pageAction(QWebPage::Forward));
    menu.addSeparator();
#ifdef _WIN32
    menu.addAction(WbActionManager::instance()->action(WbAction::COPY));
#else
    menu.addAction(pageAction(QWebEnginePage::Copy));
#endif
    menu.exec(event->globalPos());
  }
};

static const QStringList cBooks = QStringList() << "guide"
                                                << "Webots user guide"
                                                << "reference"
                                                << "Webots reference manual"
                                                << "automobile"
                                                << "Webots for automobiles";

WbDocumentation *WbDocumentation::cInstance = NULL;

WbDocumentation::WbDocumentation(QWidget *parent) : WbDockWidget(parent), mWebView(NULL) {
  assert(cInstance == NULL);
  cInstance = this;

  setWindowTitle("Documentation");
  setTabbedTitle("Documentation");
  setObjectName("Documentation");

  mWebView = new DocumentationWebView(this);
  setWidget(mWebView);

#ifdef _WIN32
  connect(mWebView, &DocumentationWebView::selectionChanged, this, &WbDocumentation::updateCopyAction);
  connect(WbActionManager::instance(), &WbActionManager::userDocumentationEditCommandReceived, this,
          &WbDocumentation::handleUserCommand);
#endif
}

WbDocumentation::~WbDocumentation() {
  cInstance = NULL;
  delete mWebView;
}

const QString WbDocumentation::book() const {
  const QString url = mWebView->url().toString();
  int start = url.lastIndexOf("&book=") + 6;
  int length = url.lastIndexOf("&page=") - start;
  return url.mid(start, length);
}

const QString WbDocumentation::page() const {
  const QString url = mWebView->url().toString();
  return url.mid(url.lastIndexOf("&page=") + 6);
}

void WbDocumentation::updateCopyAction() {
  WbActionManager::instance()->setEnabled(WbAction::COPY, !mWebView->selectedText().isEmpty());
}

void WbDocumentation::handleUserCommand(WbAction::WbActionKind actionKind) {
  if (actionKind == WbAction::COPY) {
    const QString selectedText = mWebView->selectedText();
    if (!selectedText.isEmpty())
      WbClipboard::instance()->setString(selectedText);
  }
}

void WbDocumentation::openUrlInSystemBrowser(const QString &book, const QString &page) {
  QString versionString = WbApplicationInfo::version().toString();
  versionString.replace(" revision ", "-rev");
  QString url = WbStandardPaths::cyberboticsUrl() + "/doc/" + book + "/" + page + "?version=" + versionString;

  WbDesktopServices::openUrl(url);
}
