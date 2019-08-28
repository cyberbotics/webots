#ifndef POINT_2D
#define POINT_2D

#include <QtGui/QColor>

namespace webotsQtUtils {

  class Point2D {
  public:
    Point2D(double x, double y, const QColor &color = Qt::black) : mX(x), mY(y), mColor(color) {}
    Point2D(const Point2D &copy) : mX(copy.mX), mY(copy.mY), mColor(copy.mColor) {}
    virtual ~Point2D() {}

    double x() const { return mX; }
    double y() const { return mY; }
    const QColor &color() const { return mColor; }

  private:
    Point2D &operator=(const Point2D &);  // non copyable
    double mX;
    double mY;
    QColor mColor;
  };

}  // namespace webotsQtUtils

#endif
