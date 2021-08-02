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

#include "WbWebPage.hpp"

#include "WbDesktopServices.hpp"
#include "WbLog.hpp"

#ifdef _WIN32
#include <QtNetwork/QNetworkRequest>
#endif

#ifdef _WIN32
void WbWebPage::javaScriptConsoleMessage(const QString &message, int lineNumber, const QString &sourceUrl) {
  WbLog::javascriptLogToConsole(message, lineNumber, sourceUrl);
}
#else
void WbWebPage::javaScriptConsoleMessage(JavaScriptConsoleMessageLevel level, const QString &message, int lineNumber,
                                         const QString &sourceID) {
  if (sourceID.endsWith("qwebchannel.js"))
    return;  // Do not display qwebchannel.js logs because it contains pointless logs, and issues in this files are bugs.
  WbLog::javascriptLogToConsole(message, lineNumber, sourceID);
}
#endif

bool WbWebPage::acceptNavigationRequest(
#ifdef _WIN32
  QWebFrame *frame, const QNetworkRequest &request, NavigationType type) {
  const QUrl &url = request.url();
#else
  const QUrl &url, QWebPage::NavigationType type, bool isMainFrame) {
#endif
  if (type == QWebPage::NavigationTypeLinkClicked && !url.isRelative()) {
    // Send the URL to the system default URL handler
    WbDesktopServices::openUrl(url.toString());
    return false;
  }
  return QWebPage::acceptNavigationRequest(
#ifdef _WIN32
    frame, request, type
#else
    url, type, isMainFrame
#endif
  );
}

void WbWebPage::externalLinkHovered(const QString &url) {
  mHoveredLink = url;
}

WbWebPage *WbWebPage::createWindow(QWebPage::WebWindowType type) {
  // Reimplement to not create a new window when clicking on links but open it with the system default URL handler
  WbDesktopServices::openUrl(mHoveredLink);
  return NULL;
}
