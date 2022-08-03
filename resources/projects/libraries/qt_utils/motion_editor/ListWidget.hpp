/*
 * Description:  List widget with working buttons bellow
 */

#ifndef LIST_WIDGET_HPP
#define LIST_WIDGET_HPP

#include <QtWidgets/QGroupBox>

class QListWidget;
class QPushButton;
class QGridLayout;

namespace webotsQtUtils {

  class ListWidget : public QGroupBox {
    Q_OBJECT

  public:
    ListWidget(bool showAddRemove, bool showUpDown, bool showReset, bool showDuplicate, QWidget *parent = NULL);
    virtual ~ListWidget();

    int currentIndex() const;

  protected:
    // functions to reimplement un the sub classes
    virtual int count() const = 0;
    virtual void newItemAt(int index) = 0;
    virtual void deleteItemAt(int index) = 0;
    virtual void duplicateItemAt(int index) = 0;
    virtual void swapItemWithNext(int index) = 0;

    QListWidget *mListWidget;
    QGridLayout *mMainLayout;
    QPushButton *mResetButton;

  protected slots:

  private:
    QPushButton *mUpButton;
    QPushButton *mDownButton;
    QPushButton *mDuplicateButton;
    QPushButton *mAddButton;
    QPushButton *mRemoveButton;

  private slots:
    void updateButtons();
    virtual void up();
    virtual void down();
    virtual void add();
    virtual void remove();
    virtual void duplicate();
  };
}  // namespace webotsQtUtils

#endif
