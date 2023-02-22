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

#ifndef WB_SPLASH_SCREEN_HPP
#define WB_SPLASH_SCREEN_HPP

#include <QtWidgets/QSplashScreen>

class QPainter;

class WbSplashScreen : public QSplashScreen {
  Q_OBJECT
  Q_PROPERTY(QColor backgroundGradientStartColor MEMBER mBackgroundGradientStartColor READ backgroundGradientStartColor WRITE
               setBackgroundGradientStartColor)
  Q_PROPERTY(QColor backgroundGradientEndColor MEMBER mBackgroundGradientEndColor READ backgroundGradientEndColor WRITE
               setBackgroundGradientEndColor)
  Q_PROPERTY(QColor companyColor MEMBER mCompanyColor READ companyColor WRITE setCompanyColor)
  Q_PROPERTY(QColor taglineColor MEMBER mTaglineColor READ taglineColor WRITE setTaglineColor)
  Q_PROPERTY(QColor versionColor MEMBER mVersionColor READ versionColor WRITE setVersionColor)
  Q_PROPERTY(QColor loadingColor MEMBER mLoadingColor READ loadingColor WRITE setLoadingColor)

public:
  WbSplashScreen(const QStringList &screenshots, const QString &logoFileName);
  ~WbSplashScreen();
  void drawContents(QPainter *painter) override;
  void setLiveMessage(const QString &message) { mMessage = message; }

  // qproperty methods
  const QColor &backgroundGradientStartColor() const { return mBackgroundGradientStartColor; }
  const QColor &backgroundGradientEndColor() const { return mBackgroundGradientEndColor; }
  const QColor &companyColor() const { return mCompanyColor; }
  const QColor &taglineColor() const { return mTaglineColor; }
  const QColor &versionColor() const { return mVersionColor; }
  const QColor &loadingColor() const { return mLoadingColor; }

  void setBackgroundGradientStartColor(const QColor &color) { mBackgroundGradientStartColor = color; }
  void setBackgroundGradientEndColor(const QColor &color) { mBackgroundGradientEndColor = color; }
  void setCompanyColor(const QColor &color) { mCompanyColor = color; }
  void setTaglineColor(const QColor &color) { mTaglineColor = color; }
  void setVersionColor(const QColor &color) { mVersionColor = color; }
  void setLoadingColor(const QColor &color) { mLoadingColor = color; }

private:
  QString mMessage;
  QImage mWebotsLogo;
  QImage mScreenshot;

  QColor mBackgroundGradientStartColor;
  QColor mBackgroundGradientEndColor;
  QColor mCompanyColor;
  QColor mTaglineColor;
  QColor mVersionColor;
  QColor mLoadingColor;
};

#endif  // WB_SPLASH_SCREEN_HPP
