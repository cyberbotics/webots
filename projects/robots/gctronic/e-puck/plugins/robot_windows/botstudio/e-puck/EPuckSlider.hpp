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

/*
 * Description:  Class redefining a slider
 */

#ifndef EPUCKmSlider_HPP
#define EPUCKmSlider_HPP

#include <QtCore/QObject>
#include <QtCore/QPoint>
#include <QtCore/QSize>
#include <QtWidgets/QSlider>

QT_BEGIN_NAMESPACE
class QGraphicsScene;
class QString;
class QGraphicsProxyWidget;
class QGraphicsTextItem;
QT_END_NAMESPACE

class EPuckSlider : public QObject {
  Q_OBJECT

public:
  enum { CommonSlider, RevertibleSlider, ProgressBar };
  enum { Up, Down, Left, Right, Center, None };

  EPuckSlider(QGraphicsScene *scene, const QPoint &position, qreal orientation, const QSize &size);
  virtual ~EPuckSlider();

  bool isInverted() const { return mIsInverted; }
  int index() const { return mIndex; }
  int value() const;

  void setType(int type);
  void setTextLocation(int textLocation);
  void setTextSuffix(const QString &suffix);
  void setDisplayPrefix(bool isPrefixDisplayed);
  void setInverted(bool isInverted);
  void setValue(int value);
  void setInvertedAppearance(bool inverted);
  void setEnabled(bool enabled);
  void setRange(int min, int max);
  void setTextRatio(double ratio);
  void setIndex(int index);
  void setNeutralValue(int neutralValue);
  void setIcon(const QString &fileName);

signals:
  void valueChanged();

private slots:
  void handleAction(int action);
  void updateText(int value);
  void emitValueChanged(int value);
  void sliderPressed();
  void sliderMoved(int value);
  void sliderReleased();

private:
  void update();
  void updateStyleSheet();
  void updateTextItemTransform();

  QSlider *mSlider;
  QGraphicsProxyWidget *mSliderProxy;
  QGraphicsTextItem *mTextItem;

  QGraphicsScene *mScene;
  QPoint mPosition;
  qreal mOrientation;
  QSize mSize;
  int mType;
  int mTextLocation;
  QString mSuffix;
  QString mIconFileName;
  bool mIsPrefixDisplayed;
  bool mIsInverted;
  double mRatio;
  int mIndex;
  int mNeutralValue;
  int mPreviousValue;
  bool mWasDragged;
};

#endif
