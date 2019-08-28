#ifndef LINE_2D
#define LINE_2D

#include <QtGui/QColor>

namespace webotsQtUtils {

  class Line2D {
  public:
    Line2D(double x1, double y1, double x2, double y2, const QColor &color = Qt::black) :
      mX1(x1),
      mY1(y1),
      mX2(x2),
      mY2(y2),
      mColor(color) {}
    Line2D(const Line2D &copy) : mX1(copy.mX1), mY1(copy.mY1), mX2(copy.mX2), mY2(copy.mY2), mColor(copy.mColor) {}
    virtual ~Line2D() {}

    double x1() const { return mX1; }
    double y1() const { return mY1; }
    double x2() const { return mX2; }
    double y2() const { return mY2; }
    const QColor &color() const { return mColor; }

  private:
    Line2D &operator=(const Line2D &);  // non copyable
    double mX1;
    double mY1;
    double mX2;
    double mY2;
    QColor mColor;
  };

}  // namespace webotsQtUtils

#endif
