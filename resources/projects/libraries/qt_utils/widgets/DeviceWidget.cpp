#include "DeviceWidget.hpp"
#include <devices/Device.hpp>

#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QVBoxLayout>

using namespace webotsQtUtils;

DeviceWidget::DeviceWidget(Device *device, QWidget *parent) : QWidget(parent), mDevice(device) {
  mTitleWidget = new QWidget(this);
  mTitleWidget->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Maximum);

  mTitleLabel = new QLabel(device->name(), mTitleWidget);
  mMainWidget = new QWidget(this);
  mMainWidget->setObjectName("borderedWidget");

  mTitleLayout = new QHBoxLayout(mTitleWidget);
  mTitleLayout->addWidget(mTitleLabel, 0, Qt::AlignLeft);
  mTitleWidget->setLayout(mTitleLayout);

  mVBoxLayout = new QVBoxLayout(this);
  mVBoxLayout->addWidget(mTitleWidget);
  mVBoxLayout->addWidget(mMainWidget);

  setLayout(mVBoxLayout);
}

void DeviceWidget::setTitleSuffix(const QString &suffix) {
  mTitleLabel->setText(mDevice->name() + ": " + suffix);
}
