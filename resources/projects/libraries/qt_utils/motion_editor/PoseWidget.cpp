#include "PoseWidget.hpp"

#include "AddStateDialog.hpp"
#include "Motion.hpp"
#include "MotionGlobalSettings.hpp"
#include "MotorTargetState.hpp"
#include "MotorTargetStateWidget.hpp"
#include "Pose.hpp"

#include <devices/Motor.hpp>

#include <QtGui/QColor>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QListWidget>
#include <QtWidgets/QMessageBox>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QVBoxLayout>

using namespace webotsQtUtils;

PoseWidget::PoseWidget(QWidget *parent) : ListWidget(true, false, true, false, parent), mPose(NULL) {
  mStateWidget = new MotorTargetStateWidget(this);
  mMainLayout->addWidget(mStateWidget, 1, 0);

  connect(mResetButton, SIGNAL(pressed()), this, SLOT(resetState()));
  connect(mListWidget, SIGNAL(currentRowChanged(int)), this, SLOT(manageSelectedStateChanged()));

  updateTitleAndEnable();
}

PoseWidget::~PoseWidget() {
}

void PoseWidget::changePose(Pose *pose) {
  clearPosePointer();
  mPose = pose;

  if (mPose) {
    connect(mPose, SIGNAL(destroyed()), this, SLOT(clearPosePointer()));
    connect(mPose, SIGNAL(updated(bool)), this, SLOT(updatePoseFromModel(bool)));
    connect(mPose, SIGNAL(stateUpdated(int)), this, SLOT(updateStateFromModel(int)));
    connect(mPose, SIGNAL(stateInserted(int)), this, SLOT(insertStateFromModel(int)));
    connect(mPose, SIGNAL(stateDeleted(int)), this, SLOT(deleteStateFromModel(int)));

    for (int i = 0; i < count(); i++)
      insertStateFromModel(i);
  }

  mResetButton->setEnabled(false);
  updateTitleAndEnable();
}

void PoseWidget::updateTitleAndEnable() {
  if (mPose) {
    setTitle(tr("Pose: %1").arg(mPose->name()));
    setEnabled(true);
  } else {
    setTitle(tr("Pose: unselected"));
    setEnabled(false);
  }
}

void PoseWidget::clearPosePointer() {
  mListWidget->clear();
  if (mPose) {
    disconnect(mPose, NULL, this, NULL);
    mPose = NULL;
  }
  mStateWidget->changeState(NULL);
}

int PoseWidget::count() const {
  if (mPose)
    return mPose->states().count();
  return 0;
}

void PoseWidget::newItemAt(int index) {
  if (mPose) {
    AddStateDialog asd(Motion::instance(), this);
    asd.exec();
  }
}

void PoseWidget::deleteItemAt(int index) {
  Motion *motion = Motion::instance();
  if (motion && mPose) {
    MotorTargetState *stateToDelete = mPose->states().at(index);

    bool deleteAllowed = true;
    bool atLeastOneDefinedRelatedState = motion->isSomeStateDefinedByMotorName(stateToDelete->motor()->name());

    if (atLeastOneDefinedRelatedState) {
      QMessageBox::StandardButton answer =
        QMessageBox::question(this, tr("Delete item"),
                              tr("Deleting this state will delete the other \'%2\' motor states of the motion. "
                                 "At least one \'%1\' motor state is defined in another motion pose. "
                                 "Their data will be lost. "
                                 "Would you like to proceed anyway?")
                                .arg(stateToDelete->motor()->name())
                                .arg(stateToDelete->motor()->name()),
                              QMessageBox::Ok | QMessageBox::Cancel, QMessageBox::Cancel);
      deleteAllowed = (answer == QMessageBox::Ok);
    }

    if (deleteAllowed)
      motion->deleteStatesByMotorName(stateToDelete->motor()->name());
  }
}

void PoseWidget::swapItemWithNext(int index) {
  if (mPose)
    mPose->swapStateWithNext(index);
}

void PoseWidget::resetState() {
  MotorTargetState *state = selectedState();
  if (state)
    state->reset();

  mResetButton->setEnabled(false);
}

void PoseWidget::updatePoseFromModel(bool isStatusUpdate) {
  setUpdatesEnabled(false);

  updateTitleAndEnable();

  int currentRow = mListWidget->currentRow();
  mListWidget->clear();
  for (int i = 0; i < count(); i++)
    insertStateFromModel(i);

  if (isStatusUpdate)
    mListWidget->setCurrentRow(currentRow);

  setUpdatesEnabled(true);
}

void PoseWidget::updateStateFromModel(int index) {
  if (mPose) {
    MotorTargetState *state = mPose->states().at(index);
    QListWidgetItem *i = mListWidget->item(index);
    i->setText(state->toString());
    setItemAppearance(i, state->status());
    mResetButton->setEnabled(state->status() == MotorTargetState::MODIFIED);
  } else {
    mResetButton->setEnabled(false);
  }
}

void PoseWidget::insertStateFromModel(int index) {
  if (mPose) {
    MotorTargetState *state = mPose->states().at(index);
    QListWidgetItem *i = new QListWidgetItem(state->toString());
    setItemAppearance(i, state->status());
    mListWidget->insertItem(index, i);
  }
}

void PoseWidget::setItemAppearance(QListWidgetItem *item, MotorTargetState::Status status) {
  QFont f = item->font();
  f.setBold(status == MotorTargetState::MODIFIED);
  item->setFont(f);

  QColor color("black");
  switch (status) {
    case MotorTargetState::DISABLED:
      color.setNamedColor("dimgray");
      break;
    case MotorTargetState::INVALID:
      color.setNamedColor("red");
      break;
    default:
      break;
  }
  item->setForeground(QBrush(color));
}

void PoseWidget::deleteStateFromModel(int index) {
  if (mPose)
    delete mListWidget->takeItem(index);

  // fixed strange Qt behavior:
  // the QListWidget::itemSelectionChanged() signal is never called after the deletion
  manageSelectedStateChanged();
}

MotorTargetState *PoseWidget::selectedState() {
  if (mPose) {
    int row = mListWidget->currentRow();
    if (row >= 0 && row < mPose->states().count()) {
      MotorTargetState *state = mPose->states().at(row);
      return state;
    }
  }

  return NULL;
}

void PoseWidget::manageSelectedStateChanged() {
  MotorTargetState *state = selectedState();
  mStateWidget->changeState(state);
  mResetButton->setEnabled(state && state->status() == MotorTargetState::MODIFIED);
}
