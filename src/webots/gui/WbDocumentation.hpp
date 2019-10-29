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

#ifndef WB_DOCUMENTATION_HPP
#define WB_DOCUMENTATION_HPP

#include "WbActionManager.hpp"
#include "WbDockWidget.hpp"

class DocumentationWebView;

class WbDocumentation : public WbDockWidget {
  Q_OBJECT

public:
  static WbDocumentation *instance() { return cInstance; }

  explicit WbDocumentation(QWidget *parent = NULL);
  virtual ~WbDocumentation();

  const QString book() const;
  const QString page() const;

public slots:
  void open(const QString &book, const QString &page = "index", bool visible = true);
  void openUrlInSystemBrowser(const QUrl &url);

private:
  static WbDocumentation *cInstance;
  DocumentationWebView *mWebView;

private slots:
  void handleUserCommand(WbActionManager::WbActionKind actionKind);
  void updateCopyAction();
};

#endif
