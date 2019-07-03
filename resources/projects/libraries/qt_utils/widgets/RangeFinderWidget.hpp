/*
 * Description:  Widget displaying a webots RangeFinder device
 */

#ifndef RANGE_FINDER_WIDGET_HPP
#define RANGE_FINDER_WIDGET_HPP

#include "SensorWidget.hpp"

class QLabel;
class QHBoxLayout;

namespace webotsQtUtils {
  class RangeFinderWidget : public SensorWidget {
    Q_OBJECT

  public:
    explicit RangeFinderWidget(Device *device, QWidget *parent = NULL);
    virtual ~RangeFinderWidget() {}

    void readSensors() override;

  protected slots:
    void enable(bool enable) override;

  protected:
    bool isEnabled() const override;

    QLabel *mLabel;
    QHBoxLayout *mHBox;
  };
}  // namespace webotsQtUtils

#endif
