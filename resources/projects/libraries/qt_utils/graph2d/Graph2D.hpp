#ifndef GRAPH2D_HPP
#define GRAPH2D_HPP

#include <QtCore/QList>
#include <QtCore/QRect>
#include <QtCore/QSet>
#include <QtGui/QPainter>
#include <QtWidgets/QWidget>

class QColor;

namespace webotsQtUtils {

  class Point2D;
  class Line2D;

  class Graph2D : public QWidget {
  public:
    enum { CX, CY };
    enum { MIN, MAX };

    explicit Graph2D(QWidget *parent = NULL);
    ~Graph2D();

    virtual QSize sizeHint() const;

    void addPoint2D(const Point2D &point);
    void addLine2D(const Line2D &line);
    void clear();
    void keepNPoints(int n);

    void setXRange(double min, double max) {
      mRanges[CX][MIN] = min;
      mRanges[CX][MAX] = max;
    }
    void setYRange(double min, double max) {
      mRanges[CY][MIN] = min;
      mRanges[CY][MAX] = max;
    }
    void setUpdateRangeOnClick(bool x, bool y) {
      mUpdateRangeOnClick[CX] = x;
      mUpdateRangeOnClick[CY] = y;
    }
    void setXLabel(const QString &label) { mLabels[CX] = label; }
    void setYLabel(const QString &label) { mLabels[CY] = label; }
    void setXColor(const QColor &color) { mAxisColors[CX] = color; }
    void setYColor(const QColor &color) { mAxisColors[CY] = color; }

    double xMinRange() const { return mRanges[CX][MIN]; }
    double xMaxRange() const { return mRanges[CX][MAX]; }
    double yMinRange() const { return mRanges[CY][MIN]; }
    double yMaxRange() const { return mRanges[CY][MAX]; }

    void updateRange();
    void updateXRange();
    void updateYRange();
    void extendRange();
    void extendXRange();
    void extendYRange();

  protected:
    virtual void paintEvent(QPaintEvent *);
    virtual void mousePressEvent(QMouseEvent *event);

  private:
    enum TextAlignement { CENTERED, LEFT, RIGHT, BOTTOM, TOP };

    void computeInnerRect();
    void addNewColor(const QColor &color);
    void drawGraphBackground(QPainter &painter);
    void drawLabels(QPainter &painter);
    void drawAlignedText(QPainter &painter, const QString &text, const QPoint &pos, TextAlignement uAlign,
                         TextAlignement vAlign, double angle);
    void drawMetricGrid(QPainter &painter);
    void drawPoints(QPainter &painter);
    void drawLines(QPainter &painter);
    void drawUVPoint(QPainter &painter, double u, double v);

    int xToU(double x);
    int yToV(double y);

    bool mUpdateRangeOnClick[2];
    bool computeInnerRectAlreadyCalled;
    QRect innerRect;
    QRect outerRect;
    QList<Point2D *> mPoints;
    QList<Line2D *> mLines;
    QSet<QColor *> mColors;
    QString mLabels[2];
    QColor mAxisColors[2];
    double mRanges[2][2];
  };
}  // namespace webotsQtUtils

#endif
