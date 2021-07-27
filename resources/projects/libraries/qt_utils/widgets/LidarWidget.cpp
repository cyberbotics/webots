#include "LidarWidget.hpp"

#include <devices/Device.hpp>

#include <QtWidgets/QCheckBox>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QLabel>

#include <webots/lidar.h>

#define MAX_LABEL 25  // more label in the widget is not readable

using namespace webotsQtUtils;

LidarWidget::LidarWidget(Device *device, QWidget *parent) : SensorWidget(device, parent) {
  mVBox = new QVBoxLayout(mMainWidget);
  mNumberOfLayers = wb_lidar_get_number_of_layers(mDevice->tag());
  if (mNumberOfLayers <= MAX_LABEL) {  // one label per layer
    mLabel = new QLabel *[mNumberOfLayers];
    for (int i = 0; i < mNumberOfLayers; ++i) {
      mLabel[i] = new QLabel(this);
      mLabel[i]->setAlignment(Qt::AlignCenter);
      mVBox->addWidget(mLabel[i]);
    }
  } else {  // one label for the whole image
    mLabel = new QLabel *[1];
    mLabel[0] = new QLabel(this);
    mLabel[0]->setAlignment(Qt::AlignCenter);
    mVBox->addWidget(mLabel[0]);
  }

  mPointCloudCheckBox = new QCheckBox(mTitleWidget);
  mPointCloudCheckBox->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
  mPointCloudCheckBox->setToolTip(tr("Enable point cloud mode"));
  if (wb_lidar_is_point_cloud_enabled(mDevice->tag()))
    enablePointCloud(true);
  else
    enablePointCloud(false);

  connect(mPointCloudCheckBox, SIGNAL(toggled(bool)), this, SLOT(enablePointCloud(bool)));
  mTitleLayout->insertWidget(2, mPointCloudCheckBox);

  mMainWidget->setLayout(mVBox);
}

void LidarWidget::readSensors() {
  SensorWidget::readSensors();

  WbDeviceTag tag = mDevice->tag();
  if (wb_lidar_is_point_cloud_enabled(tag))
    enablePointCloud(true);
  else
    enablePointCloud(false);

  if (wb_lidar_get_sampling_period(tag) > 0) {
    int lidarResolution = wb_lidar_get_horizontal_resolution(tag);
    double lidarRatio = (double)lidarResolution;
    unsigned char *buffer = NULL;

    if (mNumberOfLayers <= MAX_LABEL) {  // one label per layer
      buffer = new unsigned char[lidarResolution * 4];
      for (int i = 0; i < mNumberOfLayers; ++i) {
        const float *raw = wb_lidar_get_layer_range_image(tag, i);
        if (!raw || lidarResolution < 1)
          continue;
        int labelWidth = mLabel[i]->width();
        int labelHeight = mLabel[i]->height();
        double labelRatio = (double)labelWidth / (double)labelHeight;

        QImage *image;
        int k = 0, r = 0;
        float _255OverMax = 255.0f / wb_lidar_get_max_range(tag);
        while (r < lidarResolution) {
          buffer[k++] = raw[r] * _255OverMax;
          buffer[k++] = raw[r] * _255OverMax;
          buffer[k++] = raw[r++] * _255OverMax;
          buffer[k++] = 0xFF;
        }
        image = new QImage(buffer, lidarResolution, 1, 4 * lidarResolution, QImage::Format_ARGB32);
        // make sure to see high (horizontal) resolution layer
        int scale = 1;
        if (lidarResolution > 256)
          scale = lidarResolution / 256;
        QPixmap pixmap(QPixmap::fromImage(image->scaled(lidarResolution, scale)));
        if (lidarRatio > labelRatio)
          mLabel[i]->setPixmap(pixmap.scaledToWidth(labelWidth));
        else
          mLabel[i]->setPixmap(pixmap.scaledToHeight(labelHeight));
        delete image;
      }
    } else {  // one label for the whole image
      int size = lidarResolution * mNumberOfLayers;
      buffer = new unsigned char[size * 4];
      lidarRatio /= mNumberOfLayers;
      const float *raw = wb_lidar_get_range_image(tag);
      if (raw || lidarResolution > 1) {
        int labelWidth = mLabel[0]->width();
        int labelHeight = mLabel[0]->height();
        double labelRatio = (double)labelWidth / (double)labelHeight;

        QImage *image;
        int k = 0, r = 0;
        float _255OverMax = 255.0f / wb_lidar_get_max_range(tag);
        while (r < size) {
          buffer[k++] = raw[r] * _255OverMax;
          buffer[k++] = raw[r] * _255OverMax;
          buffer[k++] = raw[r++] * _255OverMax;
          buffer[k++] = 0xFF;
        }
        image = new QImage(buffer, lidarResolution, mNumberOfLayers, 4 * lidarResolution, QImage::Format_ARGB32);
        QPixmap pixmap(QPixmap::fromImage(*image));
        if (lidarRatio > labelRatio)
          mLabel[0]->setPixmap(pixmap.scaledToWidth(labelWidth));
        else
          mLabel[0]->setPixmap(pixmap.scaledToHeight(labelHeight));
        delete image;
      }
    }
    delete[] buffer;
  }
}

void LidarWidget::enable(bool enable) {
  WbDeviceTag tag = mDevice->tag();
  if (enable)
    wb_lidar_enable(tag, static_cast<int>(wb_robot_get_basic_time_step()));
  else
    wb_lidar_disable(tag);
}

void LidarWidget::enablePointCloud(bool enable) {
  if (enable) {
    mPointCloudCheckBox->setChecked(true);
    wb_lidar_enable_point_cloud(mDevice->tag());
    mPointCloudCheckBox->setText("Point cloud: enabled");
  } else {
    mPointCloudCheckBox->setChecked(false);
    wb_lidar_disable_point_cloud(mDevice->tag());
    mPointCloudCheckBox->setText("Point cloud: disabled");
  }
}

bool LidarWidget::isEnabled() const {
  WbDeviceTag tag = mDevice->tag();
  return wb_lidar_get_sampling_period(tag) > 0;
}
