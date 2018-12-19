/*
 * Description:  Pose view
 */

#ifndef POSE_WIDGET_HPP
#define POSE_WIDGET_HPP

#include "ListWidget.hpp"
#include "MotorTargetState.hpp"

class QListWidgetItem;
class QModelIndex;

namespace webotsQtUtils {

  class Pose;
  class MotorTargetStateWidget;

  class PoseWidget : public ListWidget {
    Q_OBJECT

  public:
    PoseWidget(QWidget *parent = NULL);
    virtual ~PoseWidget();

    void changePose(Pose *pose);

  protected:
    // reimplemented functions
    int count() const override;
    void newItemAt(int index) override;
    void deleteItemAt(int index) override;
    virtual void duplicateItemAt(int index) {}  // not available in PoseWidget
    void swapItemWithNext(int index) override;

  private:
    Pose *mPose;

    MotorTargetState *selectedState();
    MotorTargetStateWidget *mStateWidget;

    void updateTitleAndEnable();
    void setItemAppearance(QListWidgetItem *item, MotorTargetState::Status status);

  private slots:
    void clearPosePointer();
    void manageSelectedStateChanged();

    void updatePoseFromModel(bool isStatusUpdate);
    void updateStateFromModel(int index);
    void insertStateFromModel(int index);
    void deleteStateFromModel(int index);
    void resetState();
  };
}  // namespace webotsQtUtils

#endif
