#include "MotorTargetStateWidget.hpp"

#include "MotionGlobalSettings.hpp"
#include "MotorTargetState.hpp"

#include <devices/Motor.hpp>

#include <QtWidgets/QCheckBox>
#include <QtWidgets/QDoubleSpinBox>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QSlider>

#include <limits>

using namespace webotsQtUtils;

MotorTargetStateWidget::MotorTargetStateWidget(QWidget *parent) :
  QGroupBox(parent),
  mState(NULL),
  mMinimum(MotorTargetState::defaultMinValue()),
  mMaximum(MotorTargetState::defaultMaxValue()) {
  setObjectName("borderedGroupBox");

  QWidget *topWidget = new QWidget(this);
  QHBoxLayout *topHLayout = new QHBoxLayout(topWidget);

  mCheckBox = new QCheckBox(tr("Enabled"), topWidget);
  topHLayout->addWidget(mCheckBox);
  mSpinBox = new QDoubleSpinBox(topWidget);
  mSpinBox->setDecimals(MotionGlobalSettings::precision());
  mSpinBox->setMaximumWidth(140);
  mSpinBox->setKeyboardTracking(false);
  topHLayout->addWidget(mSpinBox);

  QWidget *bottomWidget = new QWidget(this);
  QHBoxLayout *bottomHLayout = new QHBoxLayout(bottomWidget);

  mMinLabel = new QLabel(tr("min"), bottomWidget);
  bottomHLayout->addWidget(mMinLabel);
  mSlider = new QSlider(Qt::Horizontal, bottomWidget);
  mSlider->setRange(0, 1000);
  bottomHLayout->addWidget(mSlider);
  mMaxLabel = new QLabel(tr("max"), bottomWidget);
  bottomHLayout->addWidget(mMaxLabel);

  QVBoxLayout *vLayout = new QVBoxLayout(this);
  vLayout->addWidget(topWidget);
  vLayout->addWidget(bottomWidget);

  connect(mCheckBox, SIGNAL(stateChanged(int)), this, SLOT(checkBoxToModel(int)));
  connect(mSpinBox, SIGNAL(valueChanged(double)), this, SLOT(spinBoxToModel(double)));
  connect(mSlider, SIGNAL(valueChanged(int)), this, SLOT(sliderToModel(int)));

  updateTitleAndEnable();
}

MotorTargetStateWidget::~MotorTargetStateWidget() {
}

void MotorTargetStateWidget::changeState(MotorTargetState *state) {
  clearStatePointer();
  mState = state;

  updateRange();

  if (mState) {
    connect(mState, SIGNAL(destroyed()), this, SLOT(clearStatePointer()));
    connect(mState, SIGNAL(updated()), this, SLOT(updateStateFromModel()));
  }

  updateStateFromModel();
}

void MotorTargetStateWidget::clearStatePointer() {
  if (mState) {
    disconnect(mState, NULL, this, NULL);
    mState = NULL;
  }
}

void MotorTargetStateWidget::updateRange() {
  if (mState) {
    mMinimum = mState->minValue();
    mMaximum = mState->maxValue();
  } else {
    mMinimum = MotorTargetState::defaultMinValue();
    mMaximum = MotorTargetState::defaultMaxValue();
  }

  double step = (mMaximum - mMinimum) / 1000;
  mSpinBox->setSingleStep(step);
}

void MotorTargetStateWidget::updateStateFromModel() {
  setUpdatesEnabled(false);
  if (mState && mState->isDefined()) {
    mCheckBox->blockSignals(true);
    mCheckBox->setCheckState(Qt::Checked);
    mCheckBox->blockSignals(false);

    mSpinBox->blockSignals(true);
    mSpinBox->setEnabled(true);
    // order is important. define first the range, then the value
    if (mState->isRangeUnlimited())
      mSpinBox->setRange(-std::numeric_limits<double>::max(), std::numeric_limits<double>::max());
    else
      mSpinBox->setRange(mMinimum, mMaximum);
    mSpinBox->setValue(mState->value());
    mSpinBox->blockSignals(false);

    mSlider->blockSignals(true);
    mSlider->setEnabled(true);
    valueToSlider(mState->value());
    mSlider->blockSignals(false);

    mMinLabel->setText(QString::number(mMinimum, 'g', MotionGlobalSettings::precision()));
    mMinLabel->setEnabled(true);
    mMaxLabel->setText(QString::number(mMaximum, 'g', MotionGlobalSettings::precision()));
    mMaxLabel->setEnabled(true);
  } else {
    mCheckBox->blockSignals(true);
    mCheckBox->setCheckState(Qt::Unchecked);
    mCheckBox->blockSignals(false);

    mSpinBox->blockSignals(true);
    mSpinBox->setEnabled(false);
    mSpinBox->setMinimum(0);
    mSpinBox->setValue(0);
    mSpinBox->setMaximum(0);
    mSpinBox->blockSignals(false);

    mSlider->blockSignals(true);
    mSlider->setEnabled(false);
    mSlider->setValue((mSlider->maximum() - mSlider->minimum()) / 2);
    mSlider->blockSignals(false);

    mMinLabel->setText(tr("min"));
    mMinLabel->setEnabled(false);
    mMaxLabel->setText(tr("max"));
    mMaxLabel->setEnabled(false);
  }
  updateTitleAndEnable();
  setUpdatesEnabled(true);
}

void MotorTargetStateWidget::updateTitleAndEnable() {
  if (mState) {
    setTitle(tr("State: %1").arg(mState->motor()->name()));
    setEnabled(true);
  } else {
    setTitle(tr("State: unselected"));
    setEnabled(false);
  }
}

void MotorTargetStateWidget::valueToSlider(double value) {
  double sMin = (double)mSlider->minimum();
  double sMax = (double)mSlider->maximum();

  if (mState) {
    double scaledValue = (mState->value() - mMinimum) / (mMaximum - mMinimum);  // [0, 1]
    mSlider->setValue(scaledValue * (sMax - sMin) + sMin);
  } else
    mSlider->setValue((sMax - sMin) / 2);
}

double MotorTargetStateWidget::sliderToValue() const {
  double sMin = (double)mSlider->minimum();
  double sMax = (double)mSlider->maximum();
  double sValue = (double)mSlider->value();

  double scaledValue = (sValue - sMin) / (sMax - sMin);  // [0, 1]

  return scaledValue * (mMaximum - mMinimum) + mMinimum;
}

void MotorTargetStateWidget::checkBoxToModel(int check) {
  if (mState)
    mState->setDefined((bool)check);
}

void MotorTargetStateWidget::spinBoxToModel(double value) {
  if (mState)
    mState->setValue(value);
}

void MotorTargetStateWidget::sliderToModel(int value) {
  if (mState)
    mState->setValue(sliderToValue());
}
