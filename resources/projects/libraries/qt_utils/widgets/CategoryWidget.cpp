#include "CategoryWidget.hpp"

#include <QtCore/QString>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QWidget>

using namespace webotsQtUtils;

CategoryWidget::CategoryWidget(const QString &name, QWidget *parent) : QScrollArea(parent), mName(name), mNWidgets(0) {
  setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  setWidgetResizable(true);

  mContainerWidget = new QWidget(this);
  mContainerGridLayout = new QGridLayout(this);
  mContainerWidget->setLayout(mContainerGridLayout);
  setWidget(mContainerWidget);
}

CategoryWidget::~CategoryWidget() {
}

void CategoryWidget::addWidget(QWidget *widget) {
  QSize s(380, 250);
  widget->setMinimumSize(s);
  widget->setMaximumSize(s);
  mContainerGridLayout->addWidget(widget, mNWidgets / 2, mNWidgets % 2);
  mNWidgets++;
}
