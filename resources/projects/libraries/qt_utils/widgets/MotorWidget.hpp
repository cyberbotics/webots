/*
 * Description:  Widget displaying a webots motor device
 */

#ifndef MOTOR_WIDGET_HPP
#define MOTOR_WIDGET_HPP

#include "ScalarSensorWidget.hpp"

class QSlider;
class QLabel;

namespace webotsQtUtils {
  class MotorWidget : public ScalarSensorWidget {
    Q_OBJECT

  public:
    explicit MotorWidget(Device *device, QWidget *parent = NULL);
    virtual ~MotorWidget() {}

    void setupBounds();

    void readSensors() override;
    void writeActuators() override;

  protected slots:
    void enable(bool enable) override;
    void sendCommand();

  protected:
    bool isEnabled() const override;
    double value() override;

    double minPosition() const;
    double maxPosition() const;
    double targetPosition() const;
    void setPosition(double position);

    void computeRangeLevel(double position);
    double computeMinRange() const;
    double computeMaxRange() const;
    double defaultAbsoluteRange() const;  // M_PI for rotational motors and 1 for linear motors
    bool unbounded() const { return minPosition() == maxPosition(); }

    void setSliderPosition(double value);

    QSlider *mSlider;
    int mRangeLevel;  // used when unbounded motors
    bool mCommandRequest;
    double mTargetPosition;

    QLabel *mLabel;
  };
}  // namespace webotsQtUtils

#endif
