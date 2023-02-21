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

#include "WbSplashScreen.hpp"

#include "WbApplicationInfo.hpp"
#include "WbVersion.hpp"

#include <QtCore/QTime>
#include <QtGui/QPainter>

WbSplashScreen::WbSplashScreen(const QStringList &screenshots, const QString &logoFileName) {
  QPixmap background(960, 580);
  background.fill(Qt::black);
  QSplashScreen::setPixmap(background);

  // rand is already given a fixed seed so use millisecond-time since epoch for pseudorandomness
  int randomImageIndex = QTime::currentTime().msecsSinceStartOfDay() % screenshots.size();

  mScreenshot = QImage("images:splash_images/" + screenshots.at(randomImageIndex));
  mWebotsLogo = QImage("images:" + logoFileName);
#ifdef _WIN32
  setWindowModality(Qt::ApplicationModal);
#endif
}

WbSplashScreen::~WbSplashScreen() {
}

void WbSplashScreen::drawContents(QPainter *painter) {
#ifndef __linux__
  // fix manually the dpi scaling.
  const double dotsPerInchRatio = 96.0 / logicalDpiX();
#else
  const double dotsPerInchRatio = 1.0;
#endif

  // draw left-hand background
  QLinearGradient gradient(190, 0, 190, 580);
  gradient.setColorAt(0, backgroundGradientStartColor());
  gradient.setColorAt(1, backgroundGradientEndColor());
  painter->fillRect(QRect(0, 0, 380, 580), gradient);

  // draw screenshot image
  painter->drawImage(QRect(380, 0, 580, 580), mScreenshot);

  // draw webots logo
  painter->drawImage(QRect(15, 34, 128, 120), mWebotsLogo);

  // draw application name
  QFont font;
  font = QFont("Raleway", 44 * dotsPerInchRatio, QFont::Light);
  painter->setFont(font);
  painter->setPen(companyColor());
  painter->drawText(QRect(144, 76, 228, 60), Qt::AlignCenter, "Webots");

  // draw tagline
  font = QFont("Raleway", 15 * dotsPerInchRatio, QFont::Light);
  painter->setFont(font);
  painter->setPen(taglineColor());
  painter->drawText(QRect(0, 140, 380, 100), Qt::AlignCenter, "Model. Program. Simulate. Transfer.");

  // draw version
  font = QFont("Raleway", 30 * dotsPerInchRatio, QFont::Light);
  painter->setFont(font);
  painter->setPen(versionColor());
  painter->drawText(QRect(0, 273, 380, 100), Qt::AlignCenter, WbApplicationInfo::version().toString(true, false, false));

  // then draw updating text
  font = QFont("Helvetica", 10 * dotsPerInchRatio);
  painter->setFont(font);
  painter->setPen(loadingColor());
  painter->drawText(QRect(0, 460, 380, 100), Qt::AlignHCenter | Qt::AlignBottom, mMessage);
}
