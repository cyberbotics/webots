// Copyright 1996-2019 Cyberbotics Ltd.
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

#include "WbClipboard.hpp"
#include "WbStandardPaths.hpp"

#include <QtCore/QUrl>

#include <cassert>

#include <QtGui/QContextMenuEvent>
#include <QtGui/QDesktopServices>
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
    actionManager->setEnabled(WbActionManager::COPY, false);
    actionManager->setEnabled(WbActionManager::CUT, false);
    actionManager->setEnabled(WbActionManager::PASTE, false);
    actionManager->setEnabled(WbActionManager::SELECT_ALL, false);
    actionManager->setEnabled(WbActionManager::UNDO, false);
    actionManager->setEnabled(WbActionManager::REDO, false);
    actionManager->setEnabled(WbActionManager::FIND, false);
    actionManager->setEnabled(WbActionManager::FIND_NEXT, false);
    actionManager->setEnabled(WbActionManager::FIND_PREVIOUS, false);
  }
#endif

  void contextMenuEvent(QContextMenuEvent *event) override {
    QMenu menu(this);
    menu.addAction(pageAction(QWebPage::Back));
    menu.addAction(pageAction(QWebPage::Forward));
    menu.addSeparator();
#ifdef _WIN32
    menu.addAction(WbActionManager::instance()->action(WbActionManager::COPY));
#else
    menu.addAction(pageAction(QWebEnginePage::Copy));
#endif
    menu.exec(event->globalPos());
  }

#ifndef _WIN32
  // Open the external links (for example `<a href="https://www.w3schools.com" target="_blank">`) in the system browser.
  QWebEngineView *createWindow(QWebEnginePage::WebWindowType type) override {
    // create a dummy view until the url is actually set, and remove it later.
    QWebEngineView *view = new QWebEngineView(this);
    connect(view->page(), &QWebEnginePage::urlChanged, dynamic_cast<WbDocumentation *>(parent()),
            &WbDocumentation::openUrlInSystemBrowser);
    connect(view->page(), &QWebEnginePage::urlChanged, view, &QObject::deleteLater);
    return view;
  }
#endif
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

void WbDocumentation::open(const QString &book, const QString &page, bool visible) {
  /*
  // Debug code: uncomment to show a web inspector for QtWebKit.
  static QWebInspector *inspector = NULL;
  if (inspector == NULL) {
    mWebView->settings()->setAttribute(QWebSettings::DeveloperExtrasEnabled, true);

    inspector = new QWebInspector;
    inspector->setPage(mWebView->page());

    QDialog *dialog = new QDialog();
    QVBoxLayout *layout = new QVBoxLayout(dialog);
    layout->addWidget(inspector);
    dialog->show();
  }
  */

  QUrl url = QUrl::fromLocalFile(WbStandardPaths::webotsHomePath() + "docs/index.html");
  url.setUrl(QString("%1?url=&book=%2&page=%3").arg(url.toString()).arg(book).arg(page));
  mWebView->load(url);

  for (int i = 0; i < cBooks.size(); i += 2)
    if (cBooks[i] == book) {
      setWindowTitle(cBooks[i + 1]);
      break;
    }
  if (visible)
    show();
  else
    hide();

#ifdef _WIN32
  // QtWebKit: Open external links in system browser.
  mWebView->page()->setLinkDelegationPolicy(QWebPage::DelegateExternalLinks);
  connect(mWebView->page(), &QWebPage::linkClicked, this, &WbDocumentation::openUrlInSystemBrowser, Qt::UniqueConnection);
#elif defined(__APPLE__)
  // Chromium bug on macOS:
  // - warnings like the following one are displayed in the console: "2018-05-08 11:17:37.496 QtWebEngineProcess[70885:9694859]
  // Couldn't set selectedTextBackgroundColor from default ()"
  // - reference: https://bugs.chromium.org/p/chromium/issues/detail?id=641509
  QWebEngineScript script;
  script.setSourceCode("(function() {"
                       "    css = document.createElement('style');"
                       "    css.type = 'text/css';"
                       "    document.head.appendChild(css);"
                       "    css.innerText = '::selection { background: #b2d7fe; }';"
                       "})()");
  mWebView->page()->scripts().insert(script);
#endif
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
  WbActionManager::instance()->setEnabled(WbActionManager::COPY, !mWebView->selectedText().isEmpty());
}

void WbDocumentation::handleUserCommand(WbActionManager::WbActionKind actionKind) {
  if (actionKind == WbActionManager::COPY) {
    const QString selectedText = mWebView->selectedText();
    if (!selectedText.isEmpty())
      WbClipboard::instance()->setString(selectedText);
  }
}

void WbDocumentation::openUrlInSystemBrowser(const QUrl &url) {
  QDesktopServices::openUrl(url);
}
