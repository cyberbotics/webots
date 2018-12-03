#include "CameraWidget.hpp"
#include <devices/Device.hpp>

#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QLabel>

#include <webots/camera.h>

using namespace webotsQtUtils;

CameraWidget::CameraWidget(Device *device, QWidget *parent) : SensorWidget(device, parent) {
  mHBox = new QHBoxLayout(mMainWidget);
  mLabel = new QLabel(this);
  mLabel->setAlignment(Qt::AlignCenter);

  mHBox->addWidget(mLabel);
  mMainWidget->setLayout(mHBox);
}

void CameraWidget::readSensors() {
  SensorWidget::readSensors();

  WbDeviceTag tag = mDevice->tag();
  if (wb_camera_get_sampling_period(tag) > 0) {
    int cameraWidth = wb_camera_get_width(tag);
    int cameraHeight = wb_camera_get_height(tag);
    const unsigned char *raw = wb_camera_get_image(tag);
    if (!raw || cameraWidth < 1 || cameraHeight < 1)
      return;

    int labelWidth = mLabel->width();
    int labelHeight = mLabel->height();
    double cameraRatio = (double)cameraWidth / (double)cameraHeight;
    double labelRatio = (double)labelWidth / (double)labelHeight;
    QImage *image;
    image = new QImage(raw, cameraWidth, cameraHeight, 4 * cameraWidth, QImage::Format_ARGB32);
    QPixmap pixmap(QPixmap::fromImage(*image));

    if (cameraRatio > labelRatio)
      mLabel->setPixmap(pixmap.scaledToWidth(labelWidth));
    else
      mLabel->setPixmap(pixmap.scaledToHeight(labelHeight));
    delete image;
  }
}

void CameraWidget::enable(bool enable) {
  WbDeviceTag tag = mDevice->tag();
  if (enable)
    wb_camera_enable(tag, static_cast<int>(wb_robot_get_basic_time_step()));
  else
    wb_camera_disable(tag);
}

bool CameraWidget::isEnabled() const {
  WbDeviceTag tag = mDevice->tag();
  return wb_camera_get_sampling_period(tag) > 0;
}
