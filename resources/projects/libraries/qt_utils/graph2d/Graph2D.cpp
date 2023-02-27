#include "Graph2D.hpp"
#include "Line2D.hpp"
#include "Point2D.hpp"

#include <cmath>
#include <limits>

using namespace webotsQtUtils;

static const int offset = 4;
static const int precision = 4;
static const int fontPointSize = 8;

Graph2D::Graph2D(QWidget *parent) : QWidget(parent), computeInnerRectAlreadyCalled(false) {
  setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Minimum);
  for (int i = 0; i < 2; i++) {
    mRanges[i][MIN] = std::numeric_limits<double>::infinity();
    mRanges[i][MAX] = -std::numeric_limits<double>::infinity();
    mUpdateRangeOnClick[i] = true;
  }
  mLabels[CX] = tr("Time [s]");
  mLabels[CY] = tr("Raw");
  for (int i = 0; i < 2; i++)
    mAxisColors[i] = QColor(Qt::darkGray);

  QFont f(font());
  f.setPointSize(fontPointSize);
  setFont(f);
}

Graph2D::~Graph2D() {
  clear();
}

QSize Graph2D::sizeHint() const {
  static QSize size(50, 50);
  return size;
}

void Graph2D::clear() {
  foreach (Point2D *point, mPoints)
    delete point;
  mPoints.clear();

  foreach (Line2D *line, mLines)
    delete line;
  mLines.clear();

  foreach (QColor *color, mColors)
    delete color;
  mColors.clear();
  update();
}

void Graph2D::addNewColor(const QColor &color) {
  bool found = false;
  foreach (const QColor *currentColor, mColors) {
    if (color == *currentColor) {
      found = true;
      break;
    }
  }
  if (!found)
    mColors.insert(new QColor(color));
}

void Graph2D::addPoint2D(const Point2D &point) {
  mPoints.append(new Point2D(point));
  addNewColor(point.color());
  update();
}

void Graph2D::addLine2D(const Line2D &line) {
  mLines.append(new Line2D(line));
  addNewColor(line.color());
  update();
}

void Graph2D::drawAlignedText(QPainter &painter, const QString &text, const QPoint &pos, TextAlignement uAlign,
                              TextAlignement vAlign, double angle) {
  QFontMetrics metrics(font());
  QRect textBounds = metrics.tightBoundingRect(text);

  double u = 0.0;
  double v = 0.0;

  switch (uAlign) {
    case LEFT:
      u = 0.0;
      break;
    case RIGHT:
      u = -textBounds.width();
      break;
    default:
    case CENTERED:
      u = -0.5 * textBounds.width();
      break;
  }
  switch (vAlign) {
    case TOP:
      v = textBounds.height();
      break;
    case BOTTOM:
      v = 0.0;
      break;
    default:
    case CENTERED:
      v = 0.5 * textBounds.height();
      break;
  }

  painter.save();
  painter.translate(pos.x(), pos.y());
  painter.rotate(angle);
  painter.drawText(u, v, text);
  painter.restore();
}

void Graph2D::computeInnerRect() {
  if (computeInnerRectAlreadyCalled)
    return;
  computeInnerRectAlreadyCalled = true;

  outerRect = rect();

  QFontMetrics metrics(font());
  int fontHeight = metrics.height();

  QPoint innerTopLeftPoint(fontHeight + 2 * offset, offset);
  QSize innerSize(rect().width() - innerTopLeftPoint.x() - offset,
                  rect().height() - innerTopLeftPoint.y() - (fontHeight + 2 * offset));
  innerRect = QRect(innerTopLeftPoint, innerSize);
}

void Graph2D::drawGraphBackground(QPainter &painter) {
  painter.drawRect(innerRect);
}

void Graph2D::drawLabels(QPainter &painter) {
  QString rangeText[2][2];
  for (int i = 0; i < 2; i++) {
    for (int j = 0; j < 2; j++)
      rangeText[i][j] = QString::number(mRanges[i][j], 'g', precision);
  }

  int width_2 = round(0.5 * innerRect.width());
  int height_2 = round(0.5 * innerRect.height());

  painter.setPen(Qt::darkGray);
  drawAlignedText(painter, rangeText[CX][MIN], innerRect.bottomLeft() + QPoint(0, offset), LEFT, TOP, 0.0);
  drawAlignedText(painter, rangeText[CX][MAX], innerRect.bottomRight() + QPoint(0, offset), RIGHT, TOP, 0.0);
  drawAlignedText(painter, rangeText[CY][MAX], innerRect.topLeft() + QPoint(-offset, 0), RIGHT, BOTTOM, -90.0);
  drawAlignedText(painter, rangeText[CY][MIN], innerRect.bottomLeft() + QPoint(-offset, 0), LEFT, BOTTOM, -90.0);

  painter.setPen(Qt::black);
  drawAlignedText(painter, mLabels[CX], innerRect.center() + QPoint(0, height_2 + offset), CENTERED, TOP, 0.0);
  drawAlignedText(painter, mLabels[CY], innerRect.center() + QPoint(-width_2 - offset, 0), CENTERED, BOTTOM, -90.0);
}

void Graph2D::paintEvent(QPaintEvent *event) {
  QPainter painter(this);

  computeInnerRect();

  painter.setPen(Qt::gray);
  painter.setBrush(Qt::white);
  drawGraphBackground(painter);
  drawMetricGrid(painter);
  drawLabels(painter);
  drawPoints(painter);
  drawLines(painter);
}

void Graph2D::mousePressEvent(QMouseEvent *event) {
  if (mUpdateRangeOnClick[CX])
    updateXRange();
  if (mUpdateRangeOnClick[CY])
    updateYRange();

  QWidget::mousePressEvent(event);
}

void Graph2D::updateRange() {
  updateXRange();
  updateYRange();
}

void Graph2D::extendRange() {
  extendXRange();
  extendYRange();
}

void Graph2D::extendXRange() {
  foreach (Point2D *point, mPoints) {
    if (point->x() < mRanges[CX][MIN])
      mRanges[CX][MIN] = point->x();
    if (point->x() > mRanges[CX][MAX])
      mRanges[CX][MAX] = point->x();
  }
}

void Graph2D::extendYRange() {
  foreach (Point2D *point, mPoints) {
    if (point->y() < mRanges[CY][MIN])
      mRanges[CY][MIN] = point->y();
    if (point->y() > mRanges[CY][MAX])
      mRanges[CY][MAX] = point->y();
  }
}

void Graph2D::updateXRange() {
  mRanges[CX][MIN] = std::numeric_limits<double>::infinity();
  mRanges[CX][MAX] = -std::numeric_limits<double>::infinity();
  extendXRange();
}

void Graph2D::updateYRange() {
  mRanges[CY][MIN] = std::numeric_limits<double>::infinity();
  mRanges[CY][MAX] = -std::numeric_limits<double>::infinity();
  extendYRange();
}

int Graph2D::xToU(double x) {
  double delta = mRanges[CX][MAX] - mRanges[CX][MIN];
  if (delta == 0.0)
    return 0.5 * innerRect.width() + innerRect.left();
  else
    return (x - mRanges[CX][MIN]) / (mRanges[CX][MAX] - mRanges[CX][MIN]) * innerRect.width() + innerRect.left();
}

int Graph2D::yToV(double y) {
  double delta = mRanges[CY][MAX] - mRanges[CY][MIN];
  if (delta == 0.0)
    return 0.5 * innerRect.height() + innerRect.top();
  else
    return (1.0 - (y - mRanges[CY][MIN]) / delta) * innerRect.height() + innerRect.top();
}

void Graph2D::drawPoints(QPainter &painter) {
  foreach (QColor *color, mColors) {
    painter.setPen(*color);
    foreach (Point2D *point, mPoints) {
      if (point->color() == *color && point->x() <= mRanges[CX][MAX] && point->x() >= mRanges[CX][MIN] &&
          point->y() <= mRanges[CY][MAX] && point->y() >= mRanges[CY][MIN])
        drawUVPoint(painter, xToU(point->x()), yToV(point->y()));
    }
  }
}

void Graph2D::drawLines(QPainter &painter) {
  foreach (QColor *color, mColors) {
    painter.setPen(*color);
    foreach (Line2D *line, mLines) {
      if (line->color() == *color)
        painter.drawLine(xToU(line->x1()), yToV(line->y1()), xToU(line->x2()), yToV(line->y2()));
    }
  }
}

void Graph2D::drawUVPoint(QPainter &painter, double u, double v) {
  painter.drawLine(u, v - 1, u, v + 1);
  painter.drawLine(u - 1, v, u + 1, v);
}

void Graph2D::keepNPoints(int n) {
  while (mPoints.size() > n)
    delete mPoints.takeFirst();
}

void Graph2D::drawMetricGrid(QPainter &painter) {
  // vertical lines
  painter.setPen(mAxisColors[CY].lighter(125));
  double deltaX = mRanges[CX][MAX] - mRanges[CX][MIN];
  deltaX *= 0.999;  // in order to decrease the order of magnitude if deltaX is a perfect divider of incrementX
  double orderOfMagnitudeX = floor(log10(deltaX));
  double incrementX = pow(10, orderOfMagnitudeX);
  double firstValueX = ceil(mRanges[CX][MIN] / incrementX) * incrementX;
  for (double s = firstValueX; s < mRanges[CX][MAX]; s += incrementX) {
    QPoint p1(xToU(s), yToV(mRanges[CY][MIN]));
    QPoint p2(xToU(s), yToV(mRanges[CY][MAX]));
    painter.drawLine(p1, p2);
  }

  // horizontal lines
  painter.setPen(mAxisColors[CX].lighter(125));
  double deltaY = mRanges[CY][MAX] - mRanges[CY][MIN];
  deltaY *= 0.999;  // in order to decrease the order of magnitude if deltaY is a perfect divider of incrementY
  double orderOfMagnitudeY = floor(log10(deltaY));
  double incrementY = pow(10, orderOfMagnitudeY);
  double firstValueY = ceil(mRanges[CY][MIN] / incrementY) * incrementY;
  for (double t = firstValueY; t < mRanges[CY][MAX]; t += incrementY) {
    QPoint p1(xToU(mRanges[CX][MIN]), yToV(t));
    QPoint p2(xToU(mRanges[CX][MAX]), yToV(t));
    painter.drawLine(p1, p2);
  }

  // s=0 line (can override a previsouly drawn line)
  painter.setPen(mAxisColors[CY]);
  if (0.0 > mRanges[CX][MIN] && 0.0 < mRanges[CX][MAX]) {
    QPoint pX1(xToU(0.0), yToV(mRanges[CY][MIN]));
    QPoint pX2(xToU(0.0), yToV(mRanges[CY][MAX]));
    painter.drawLine(pX1, pX2);
  }
  // t=0 line (can override a previsouly drawn line)
  painter.setPen(mAxisColors[CX]);
  if (0.0 > mRanges[CY][MIN] && 0.0 < mRanges[CY][MAX]) {
    QPoint pT1(xToU(mRanges[CX][MIN]), yToV(0.0));
    QPoint pT2(xToU(mRanges[CX][MAX]), yToV(0.0));
    painter.drawLine(pT1, pT2);
  }
}
