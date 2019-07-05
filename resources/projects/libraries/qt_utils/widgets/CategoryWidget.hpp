/*
 * Description:  Sort widgets in a grid of 2 columns
 */

#ifndef CATEGORY_WIDGET_HPP
#define CATEGORY_WIDGET_HPP

#include <QtCore/QList>
#include <QtWidgets/QScrollArea>

class QGridLayout;
class QString;

namespace webotsQtUtils {

  class CategoryWidget : public QScrollArea {
  public:
    explicit CategoryWidget(const QString &name, QWidget *parent = NULL);
    virtual ~CategoryWidget();

    const QString &name() const { return mName; }
    void addWidget(QWidget *widget);

  private:
    QString mName;
    int mNWidgets;

    QWidget *mContainerWidget;
    QGridLayout *mContainerGridLayout;
  };
}  // namespace webotsQtUtils

#endif
