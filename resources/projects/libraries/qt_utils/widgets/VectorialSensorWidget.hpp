/*
 * Description:  Widget displaying a webots vectorial sensor device
 */

#ifndef VECTORIAL_SENSOR_WIDGET_HPP
#define VECTORIAL_SENSOR_WIDGET_HPP

#include "SensorWidget.hpp"

#include <QtCore/QList>

class QHBoxLayout;
class QComboBox;
class QVector4D;

namespace webotsQtUtils {
  class Graph2D;

  class VectorialSensorWidget : public SensorWidget {
    Q_OBJECT

  public:
    explicit VectorialSensorWidget(Device *device, QWidget *parent = NULL);
    virtual ~VectorialSensorWidget();

    void readSensors() override;

  protected slots:
    void setMode(int mode);

  protected:
    virtual const double *values() = 0;

    enum { XY, YZ, XZ, TIME };

    void displayVector(QVector4D *vector);
    void removeOldVectors();
    void removeAllVectors();

    Graph2D *mGraph2D;
    QHBoxLayout *mHBoxLayout;
    QComboBox *mComboBox;
    int mMode;
    QList<QVector4D *> mVectorList;
  };
}  // namespace webotsQtUtils

#endif
