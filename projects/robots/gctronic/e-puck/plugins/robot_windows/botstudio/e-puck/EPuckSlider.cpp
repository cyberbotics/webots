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
 * Description:  Implementation of the EPuckSlider.hpp functions
 */

#include "EPuckSlider.hpp"

#include <QtWidgets/QGraphicsProxyWidget>
#include <QtWidgets/QGraphicsScene>
#include <QtWidgets/QGraphicsTextItem>
#include <core/BotStudioPaths.hpp>

#include <limits>

EPuckSlider::EPuckSlider(QGraphicsScene *scene, const QPoint &position, qreal orientation, const QSize &size) :
  mScene(scene),
  mPosition(position),
  mOrientation(orientation),
  mSize(size),
  mType(CommonSlider),
  mTextLocation(None),
  mSuffix("%"),
  mIconFileName(),
  mIsPrefixDisplayed(false),
  mIsInverted(false),
  mRatio(1.0),
  mIndex(-1),
  mNeutralValue(std::numeric_limits<int>::max()) {
  mSlider = new QSlider(Qt::Horizontal);
  mSlider->setFixedSize(size);
  mSlider->setValue(0);
  mSlider->setPageStep(0);

  mWasDragged = false;
  mPreviousValue = -1;

  mSliderProxy = mScene->addWidget(mSlider);
  QTransform t;
  t.translate(position.x(), position.y());
  t.rotate(orientation);
  t.translate(-size.width() / 2, -size.height() / 2);
  mSliderProxy->setTransform(t);

  mTextItem = new QGraphicsTextItem();
  scene->addItem(mTextItem);
  updateText(mSlider->value());

  connect(mSlider, SIGNAL(actionTriggered(int)), this, SLOT(handleAction(int)));
  connect(mSlider, SIGNAL(valueChanged(int)), this, SLOT(updateText(int)));
  connect(mSlider, SIGNAL(valueChanged(int)), this, SLOT(emitValueChanged(int)));
  connect(mSlider, SIGNAL(sliderPressed()), this, SLOT(sliderPressed()));
  connect(mSlider, SIGNAL(sliderMoved(int)), this, SLOT(sliderMoved(int)));
  connect(mSlider, SIGNAL(sliderReleased()), this, SLOT(sliderReleased()));

  updateStyleSheet();
}

void EPuckSlider::setType(int type) {
  mType = type;
  update();
}

void EPuckSlider::setTextLocation(int textLocation) {
  mTextLocation = textLocation;
  update();
}

void EPuckSlider::setTextSuffix(const QString &suffix) {
  mSuffix = suffix;
  update();
}

void EPuckSlider::setDisplayPrefix(bool isPrefixDisplayed) {
  mIsPrefixDisplayed = isPrefixDisplayed;
  update();
}

void EPuckSlider::setIcon(const QString &fileName) {
  mIconFileName = fileName;
}

void EPuckSlider::setInverted(bool isInverted) {
  if (mIsInverted != isInverted) {
    mIsInverted = isInverted;
    emit valueChanged();
    update();
  }
}

void EPuckSlider::update() {
  updateStyleSheet();
  updateText(mSlider->value());
}

void EPuckSlider::emitValueChanged(int value) {
  emit valueChanged();
}

int EPuckSlider::value() const {
  return mSlider->value();
}

void EPuckSlider::updateText(int value) {
  if (mTextLocation == None) {
    mTextItem->setPlainText("");
    return;
  }

  QString s;

  if (!mIconFileName.isEmpty())
    s += "<img src=\"" + BotStudioPaths::getIconsPath() + mIconFileName + "\"/>: ";

  if (value == mNeutralValue)
    s += "ND";
  else {
    if (mIsPrefixDisplayed) {
      if (mSlider->invertedAppearance()) {
        if (mIsInverted)
          s += "&lt;";
        else
          s += "&gt;";
      } else {
        if (mIsInverted)
          s += "&gt;";
        else
          s += "&lt;";
      }
    }
    s += QString::number(static_cast<double>(value) * mRatio, 'g', 3);
    s += mSuffix;
  }

  mTextItem->setHtml(s);

  updateTextItemTransform();
}

void EPuckSlider::updateTextItemTransform() {
  if (mTextLocation == None)
    return;

  mTextItem->resetTransform();

  QRectF r = mTextItem->sceneBoundingRect();

  QTransform t;
  t.translate(mPosition.x(), mPosition.y());
  t.rotate(mOrientation);

  int offset = 18;
  switch (mTextLocation) {
    case Up:
      t.translate(0, -mSize.height() / 2 - offset);
      break;
    case Down:
      t.translate(0, mSize.height() / 2 + offset);
      break;
    case Left:
      t.translate(-mSize.width() / 2 - offset, 0);
      break;
    case Right:
      t.translate(mSize.width() / 2 + offset, 0);
      break;
    default:
      break;
  }

  t.rotate(-mOrientation);
  t.translate(-r.width() / 2, -r.height() / 2);

  mTextItem->setTransform(t);
}

EPuckSlider::~EPuckSlider() {
  mScene->removeItem(mSliderProxy);
  mScene->removeItem(mTextItem);
  delete mSlider;
  delete mTextItem;
}

void EPuckSlider::sliderPressed() {
  mWasDragged = false;
}

void EPuckSlider::sliderMoved(int value) {
  mWasDragged = true;
  mPreviousValue = value;
}

void EPuckSlider::sliderReleased() {
  if (mType == RevertibleSlider && !mWasDragged) {
    const int v = mSlider->value();
    if (v == mPreviousValue) {
      setInverted(!isInverted());
      update();
    }
    mPreviousValue = v;
  }
}

void EPuckSlider::handleAction(int action) {
  if (mType == RevertibleSlider) {
    if (mSlider->invertedAppearance()) {
      if (action == QSlider::SliderPageStepAdd)
        setInverted(false);
      else if (action == QSlider::SliderPageStepSub)
        setInverted(true);
    } else {
      if (action == QSlider::SliderPageStepAdd)
        setInverted(true);
      else if (action == QSlider::SliderPageStepSub)
        setInverted(false);
    }
  }
  update();
}

void EPuckSlider::setValue(int value) {
  mSlider->blockSignals(true);
  mSlider->setValue(value);
  mSlider->blockSignals(false);
  update();
}

void EPuckSlider::updateStyleSheet() {
  QString mystyle(
    "QSlider::groove:horizontal {"
    "  border: 1px solid black;"
    "  background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #B1B1B1, stop:1 #c4c4c4);"
    "  left: 2px; right: 2px;"
    "  top: 2px; bottom: 2px;"
    "}"
    "QSlider::handle:horizontal {"
    "  background: qlineargradient(x1:0, y1:0, x2:1, y2:1, stop:0 #b4b4b4, stop:1 #8f8f8f);"
    "  border: 1px solid #5c5c5c;"
    "  width: 6px;"
    "  margin: -2px 0; /* handle is placed by default on the contents rect of the groove. Expand outside the groove */"
    "  border-radius: 3px;"
    "}"
    "QSlider::handle:horizontal:disabled {"
    "  width: 1px;"
    "  border: 0px;"
    "  background: black;"
    "}"
    "QSlider::add-page:horizontal {"
    "  background: templateA;"
    "}"
    "QSlider::sub-page:horizontal {"
    "  background: templateB;"
    "}");

  if (mType == RevertibleSlider) {
    if (mIsInverted) {
      mystyle = mystyle.replace("templateA", "red");
      mystyle = mystyle.replace("templateB", "lightgreen");
    } else {
      mystyle = mystyle.replace("templateA", "lightgreen");
      mystyle = mystyle.replace("templateB", "red");
    }
  } else if (mType == ProgressBar) {
    mystyle = mystyle.replace("templateA", "lightgreen");
    mystyle = mystyle.replace("templateB", "red");
  } else {
    mystyle = mystyle.replace("templateA", "white");
    mystyle = mystyle.replace("templateB", "white");
  }

  mSlider->setStyleSheet(mystyle);
}

void EPuckSlider::setInvertedAppearance(bool inverted) {
  mSlider->setInvertedAppearance(inverted);
}

void EPuckSlider::setEnabled(bool enabled) {
  mSlider->setEnabled(enabled);
}

void EPuckSlider::setRange(int min, int max) {
  mSlider->setRange(min, max);
  update();
}

void EPuckSlider::setTextRatio(double ratio) {
  mRatio = ratio;
  update();
}

void EPuckSlider::setIndex(int index) {
  mIndex = index;
}

void EPuckSlider::setNeutralValue(int neutralValue) {
  mNeutralValue = neutralValue;
}
