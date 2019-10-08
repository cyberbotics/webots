/*
 * Description:  Pose editor dialog box
 */

#ifndef ADD_STATE_DIALOG_HPP
#define ADD_STATE_DIALOG_HPP

#include <QtWidgets/QDialog>

class QListWidget;

namespace webotsQtUtils {
  class Motion;

  class AddStateDialog : public QDialog {
    Q_OBJECT

  public:
    explicit AddStateDialog(Motion *motion, QWidget *parent = NULL);
    virtual ~AddStateDialog();

  private slots:
    void ok();

  private:
    void populateListWidget();

    QListWidget *mListWidget;
    Motion *mMotion;
  };
}  // namespace webotsQtUtils

#endif
