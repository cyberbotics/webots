/*
 * Description:  Motion view
 */

#ifndef MOTION_WIDGET_HPP
#define MOTION_WIDGET_HPP

#include "ListWidget.hpp"
#include "Pose.hpp"

class QListWidgetItem;
class QModelIndex;

namespace webotsQtUtils {

  class Motion;
  class PoseWidget;

  class MotionWidget : public ListWidget {
    Q_OBJECT

  public:
    MotionWidget(QWidget *parent = NULL);
    virtual ~MotionWidget();

    void setMotion(Motion *motion);

  signals:
    void selectedPoseChanged();

  protected:
    // reimplemented functions
    virtual int count() const;
    virtual void newItemAt(int index);
    virtual void deleteItemAt(int index);
    virtual void duplicateItemAt(int index);
    virtual void swapItemWithNext(int index);

  private:
    Pose *selectedPose() const;
    void setItemAppearance(QListWidgetItem *item, Pose::Status status);

    Motion *mMotion;
    PoseWidget *mPoseWidget;

  private slots:
    void updateTitle();
    void selectPoseFromModel(int index);
    void updatePoseFromModel(int index);
    void insertPoseFromModel(int index);
    void deletePoseFromModel(int index);

    void clearMotionPointer();
    void manageSelectedPoseChanged();

    void manageDoubleClick(const QModelIndex &index);
  };
}  // namespace webotsQtUtils

#endif
