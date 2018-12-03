/*
 * Description:  Widget displaying a webots Lidar device
 */

#ifndef LIDAR_WIDGET_HPP
#define LIDAR_WIDGET_HPP

#include "SensorWidget.hpp"

class QLabel;
class QHBoxLayout;

namespace webotsQtUtils {
  class LidarWidget : public SensorWidget {
    Q_OBJECT

  public:
    LidarWidget(Device *device, QWidget *parent = NULL);
    virtual ~LidarWidget() {}

    virtual void readSensors();

  protected slots:
    virtual void enable(bool enable);
    void enablePointCloud(bool enable);

  protected:
    bool isEnabled() const;

    QLabel **mLabel;
    QVBoxLayout *mVBox;
    QCheckBox *mPointCloudCheckBox;
    QLabel *mPointCloudLabel;
    int mNumberOfLayers;
  };
}  // namespace webotsQtUtils

#endif
