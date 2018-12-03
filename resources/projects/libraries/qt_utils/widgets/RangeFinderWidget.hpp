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
    RangeFinderWidget(Device *device, QWidget *parent = NULL);
    virtual ~RangeFinderWidget() {}

    virtual void readSensors();

  protected slots:
    virtual void enable(bool enable);

  protected:
    bool isEnabled() const;

    QLabel *mLabel;
    QHBoxLayout *mHBox;
  };
}  // namespace webotsQtUtils

#endif
