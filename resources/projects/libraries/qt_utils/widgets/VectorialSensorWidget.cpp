#include "VectorialSensorWidget.hpp"
#include <devices/Device.hpp>
#include "CommonProperties.hpp"

#include "../graph2d/Graph2D.hpp"
#include "../graph2d/Point2D.hpp"

#include <QtGui/QVector4D>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QHBoxLayout>

using namespace webotsQtUtils;

static const QColor black = QColor(0, 0, 0);
static const QColor red = QColor(200, 50, 50);
static const QColor green = QColor(50, 200, 50);
static const QColor blue = QColor(50, 50, 200);

VectorialSensorWidget::VectorialSensorWidget(Device *device, QWidget *parent) : SensorWidget(device, parent), mMode(TIME) {
  mHBoxLayout = new QHBoxLayout(mMainWidget);
  mGraph2D = new Graph2D(mMainWidget);

  mHBoxLayout->addWidget(mGraph2D, 0);
  mMainWidget->setLayout(mHBoxLayout);

  mComboBox = new QComboBox(mTitleWidget);
  mComboBox->addItem("XY");
  mComboBox->addItem("YZ");
  mComboBox->addItem("XZ");
  mComboBox->addItem(tr("Time"));
  mComboBox->setCurrentIndex(3);
  mComboBox->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);

  mTitleLayout->addWidget(mComboBox);

  connect(mComboBox, SIGNAL(currentIndexChanged(int)), this, SLOT(setMode(int)));
}

VectorialSensorWidget::~VectorialSensorWidget() {
  removeAllVectors();
}

void VectorialSensorWidget::readSensors() {
  SensorWidget::readSensors();
  if (isEnabled()) {
    const double *v = values();
    double time = wb_robot_get_time();
    QVector4D *v4 = new QVector4D(v[0], v[1], v[2], time);
    mVectorList.append(v4);
    displayVector(v4);
    removeOldVectors();
    QString suffix("\n{");
    suffix += QString::number(v[0], 'g', CommonProperties::precision());
    suffix += ", ";
    suffix += QString::number(v[1], 'g', CommonProperties::precision());
    suffix += ", ";
    suffix += QString::number(v[2], 'g', CommonProperties::precision());
    suffix += "}";
    setTitleSuffix(suffix);
  }
}

void VectorialSensorWidget::setMode(int mode) {
  mMode = mode;

  switch (mMode) {
    case XY:
      mGraph2D->setXLabel("X");
      mGraph2D->setYLabel("Y");
      mGraph2D->setXColor(red);
      mGraph2D->setYColor(green);
      break;
    case XZ:
      mGraph2D->setXLabel("X");
      mGraph2D->setYLabel("Z");
      mGraph2D->setXColor(red);
      mGraph2D->setYColor(blue);
      break;
    case YZ:
      mGraph2D->setXLabel("Y");
      mGraph2D->setYLabel("Z");
      mGraph2D->setXColor(green);
      mGraph2D->setYColor(blue);
      break;
    case TIME:
      mGraph2D->setXLabel(tr("time [s]"));
      mGraph2D->setYLabel(tr("raw"));
      mGraph2D->setXColor(black);
      mGraph2D->setYColor(black);
      break;
  }

  mGraph2D->clear();
  mGraph2D->updateRange();
  foreach (QVector4D *vector, mVectorList)
    displayVector(vector);
  mGraph2D->updateRange();
}

void VectorialSensorWidget::displayVector(QVector4D *vector) {
  switch (mMode) {
    case XY:
      mGraph2D->addPoint2D(Point2D(vector->x(), vector->y()));
      break;
    case XZ:
      mGraph2D->addPoint2D(Point2D(vector->x(), vector->z()));
      break;
    case YZ:
      mGraph2D->addPoint2D(Point2D(vector->y(), vector->z()));
      break;
    case TIME:
      mGraph2D->addPoint2D(Point2D(vector->w(), vector->x(), red));
      mGraph2D->addPoint2D(Point2D(vector->w(), vector->y(), green));
      mGraph2D->addPoint2D(Point2D(vector->w(), vector->z(), blue));
      break;
  }

  switch (mMode) {
    case XY:
    case XZ:
    case YZ:
      mGraph2D->extendRange();
      break;
    case TIME:
      mGraph2D->updateXRange();
      mGraph2D->extendYRange();
      break;
  }
}

void VectorialSensorWidget::removeOldVectors() {
  int n = CommonProperties::historySize();
  if (mMode == TIME)
    n *= 3;

  mGraph2D->keepNPoints(n);
  while (mVectorList.size() > n)
    delete mVectorList.takeFirst();
}

void VectorialSensorWidget::removeAllVectors() {
  mGraph2D->clear();
  foreach (QVector4D *vector, mVectorList)
    delete vector;
  mVectorList.clear();
}
