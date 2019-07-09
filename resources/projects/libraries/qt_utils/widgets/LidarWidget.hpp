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
    explicit LidarWidget(Device *device, QWidget *parent = NULL);
    virtual ~LidarWidget() {}

    void readSensors() override;

  protected slots:
    void enable(bool enable) override;
    void enablePointCloud(bool enable);

  protected:
    bool isEnabled() const override;

    QLabel **mLabel;
    QVBoxLayout *mVBox;
    QCheckBox *mPointCloudCheckBox;
    QLabel *mPointCloudLabel;
    int mNumberOfLayers;
  };
}  // namespace webotsQtUtils

#endif
