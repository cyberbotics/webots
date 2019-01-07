/*
 * Description:  Widget displaying a webots differential wheels
 */

#ifndef DIFFERENTIAL_WHEELS_WIDGET_HPP
#define DIFFERENTIAL_WHEELS_WIDGET_HPP

#include "SensorWidget.hpp"

class QPushButton;
class QGridLayout;
class QHBoxLayout;

namespace webotsQtUtils {
  class Graph2D;

  class DifferentialWheelsWidget : public SensorWidget {
    Q_OBJECT

  public:
    DifferentialWheelsWidget(Device *device, QWidget *parent = NULL);
    virtual ~DifferentialWheelsWidget();

    void readSensors() override;
    void writeActuators() override;

  protected slots:
    void enable(bool enable) override;
    void sendCommand();
    void sendStopCommand();

  protected:
    bool isEnabled() const override;
    bool isAButtonDown();

    Graph2D *mGraph2D;
    QPushButton *mPushButtons[9];
    QGridLayout *mGridLayout;
    QHBoxLayout *mHBoxLayout;
    QWidget *mButtonWidget;

    double mTargetSpeeds[2];
    double mMaxSpeed;
    bool mCommandRequest;
  };
}  // namespace webotsQtUtils

#endif
