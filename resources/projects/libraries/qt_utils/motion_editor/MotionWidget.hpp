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
    explicit MotionWidget(QWidget *parent = NULL);
    virtual ~MotionWidget();

    void setMotion(Motion *motion);

  signals:
    void selectedPoseChanged();

  protected:
    // reimplemented functions
    int count() const override;
    void newItemAt(int index) override;
    void deleteItemAt(int index) override;
    void duplicateItemAt(int index) override;
    void swapItemWithNext(int index) override;

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
