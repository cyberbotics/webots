#include "SensorWidget.hpp"
#include <devices/Device.hpp>

#include <QtWidgets/QCheckBox>
#include <QtWidgets/QHBoxLayout>

using namespace webotsQtUtils;

SensorWidget::SensorWidget(Device *device, QWidget *parent) : DeviceWidget(device, parent) {
  mCheckBox = new QCheckBox(mTitleWidget);
  mCheckBox->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
  mCheckBox->setToolTip(tr("Enable the %1 sensor").arg(mDevice->name()));

  connect(mCheckBox, &QCheckBox::toggled, this, &SensorWidget::enable);

  mTitleLayout->insertWidget(0, mCheckBox);
}

void SensorWidget::readSensors() {
  DeviceWidget::readSensors();

  mCheckBox->blockSignals(true);
  if (isEnabled()) {
    setTitleSuffix(tr("Enabled"));
    mCheckBox->setCheckState(Qt::Checked);
  } else {
    setTitleSuffix(tr("Disabled"));
    mCheckBox->setCheckState(Qt::Unchecked);
  }
  mCheckBox->blockSignals(false);
}
