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

#include "WbWebPage.hpp"

#include "WbLog.hpp"

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
