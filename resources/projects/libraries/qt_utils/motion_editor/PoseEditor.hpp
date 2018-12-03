/*
 * Description:  Pose editor dialog box
 */

#ifndef POSE_EDITOR_HPP
#define POSE_EDITOR_HPP

#include <QtWidgets/QDialog>

class QLineEdit;
class QSpinBox;

namespace webotsQtUtils {
  class Pose;

  class PoseEditor : public QDialog {
    Q_OBJECT

  public:
    PoseEditor(Pose *pose, Pose *previousPose, Pose *nextPose, bool fixedStep, QWidget *parent = NULL);
    virtual ~PoseEditor();

  private slots:
    void ok();

  private:
    Pose *mPose;
    Pose *mPreviousPose;
    Pose *mNextPose;

    bool mFixedStep;

    QLineEdit *mNameLineEdit;
    QSpinBox *mMinutesSpinBox;
    QSpinBox *mSecondsSpinBox;
    QSpinBox *mMilliSecondsSpinBox;
  };
}  // namespace webotsQtUtils

#endif
