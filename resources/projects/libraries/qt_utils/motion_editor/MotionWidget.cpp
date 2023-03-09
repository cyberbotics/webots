#include "MotionWidget.hpp"

#include "Motion.hpp"
#include "Pose.hpp"
#include "PoseEditor.hpp"
#include "PoseWidget.hpp"

#include <QtWidgets/QGridLayout>
#include <QtWidgets/QListWidget>

#include <assert.h>

using namespace webotsQtUtils;

MotionWidget::MotionWidget(QWidget *parent) : ListWidget(true, true, false, true, parent), mMotion(NULL) {
  updateTitle();
  setEnabled(false);

  mPoseWidget = new PoseWidget(this);
  mMainLayout->addWidget(mPoseWidget, 0, 1);

  connect(mListWidget, SIGNAL(doubleClicked(const QModelIndex &)), this, SLOT(manageDoubleClick(const QModelIndex &)));
  connect(mListWidget, SIGNAL(itemSelectionChanged()), this, SLOT(manageSelectedPoseChanged()));
  connect(mListWidget, SIGNAL(itemSelectionChanged()), this, SIGNAL(selectedPoseChanged()));
}

MotionWidget::~MotionWidget() {
}

void MotionWidget::setMotion(Motion *motion) {
  clearMotionPointer();
  mMotion = motion;
  mMotion->blockPoseSelection(true);

  updateTitle();
  setEnabled(true);

  connect(mMotion, SIGNAL(destroyed()), this, SLOT(clearMotionPointer()));
  connect(mMotion, SIGNAL(updated()), this, SLOT(updateTitle()));
  connect(mMotion, SIGNAL(poseSelected(int)), this, SLOT(selectPoseFromModel(int)));
  connect(mMotion, SIGNAL(poseUpdated(int)), this, SLOT(updatePoseFromModel(int)));
  connect(mMotion, SIGNAL(poseInserted(int)), this, SLOT(insertPoseFromModel(int)));
  connect(mMotion, SIGNAL(poseDeleted(int)), this, SLOT(deletePoseFromModel(int)));

  for (int i = 0; i < count(); i++)
    insertPoseFromModel(i);

  mMotion->blockPoseSelection(false);
}

Pose *MotionWidget::selectedPose() const {
  if (mMotion) {
    int row = mListWidget->currentRow();
    if (row >= 0 && row < mMotion->poses().count()) {
      Pose *pose = mMotion->poses().at(row);
      return pose;
    }
  }
  return NULL;
}

void MotionWidget::clearMotionPointer() {
  mListWidget->clear();
  if (mMotion) {
    disconnect(mMotion, NULL, this, NULL);
    mMotion = NULL;

    setEnabled(false);
  }

  updateTitle();
}

int MotionWidget::count() const {
  if (mMotion)
    return mMotion->poses().count();
  else
    return 0;
}

void MotionWidget::newItemAt(int index) {
  assert(mMotion);
  mMotion->newPoseAt(index);
}

void MotionWidget::deleteItemAt(int index) {
  assert(mMotion);
  mMotion->deletePoseAt(index);
}

void MotionWidget::duplicateItemAt(int index) {
  assert(mMotion);
  mMotion->duplicatePoseAt(index);
}

void MotionWidget::swapItemWithNext(int index) {
  assert(mMotion);
  mMotion->swapPoseWithNext(index);
}

void MotionWidget::updateTitle() {
  if (mMotion)
    setTitle(QString("Motion: %1").arg(mMotion->name()));
  else
    setTitle("Motion: unloaded");
}

void MotionWidget::selectPoseFromModel(int index) {
  mListWidget->setCurrentRow(index);
}

void MotionWidget::updatePoseFromModel(int index) {
  Pose *pose = mMotion->poses().at(index);
  QListWidgetItem *i = mListWidget->item(index);
  i->setText(pose->toString());
  setItemAppearance(i, pose->status());
}

void MotionWidget::insertPoseFromModel(int index) {
  Pose *pose = mMotion->poses().at(index);
  QListWidgetItem *i = new QListWidgetItem(pose->toString());
  setItemAppearance(i, pose->status());
  mListWidget->insertItem(index, i);

  // select the new item
  if (!mMotion->isPoseSelectionBlocked() && index < mListWidget->count())
    mListWidget->setCurrentRow(index);
}

void MotionWidget::setItemAppearance(QListWidgetItem *item, Pose::Status status) {
  QFont f = item->font();
  f.setBold(status == Pose::MODIFIED);
  item->setFont(f);

  QColor color("black");
  if (status == Pose::INVALID)
    color.setNamedColor("red");
  item->setForeground(QBrush(color));
}

void MotionWidget::deletePoseFromModel(int index) {
  delete mListWidget->takeItem(index);

  // fixed strange Qt behavior:
  // the QListWidget::itemSelectionChanged() signal is never called after the deletion
  manageSelectedPoseChanged();
}

void MotionWidget::manageDoubleClick(const QModelIndex &index) {
  int row = index.row();
  Pose *previousPose = NULL, *afterPose = NULL;
  Pose *pose = mMotion->poses().at(row);

  if (row > 0)
    previousPose = mMotion->poses().at(row - 1);
  if (row < mMotion->poses().count() - 1)
    afterPose = mMotion->poses().at(row + 1);

  PoseEditor pe(pose, previousPose, afterPose, mMotion->fixedStep(), this);
  pe.exec();
}

void MotionWidget::manageSelectedPoseChanged() {
  Pose *pose = selectedPose();
  mMotion->applyPoseToRobot(pose);
  mPoseWidget->changePose(pose);
}
