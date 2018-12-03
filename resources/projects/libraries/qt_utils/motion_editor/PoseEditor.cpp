#include "PoseEditor.hpp"

#include "Pose.hpp"

#include <QtWidgets/QFormLayout>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QMessageBox>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSpinBox>

#include <cassert>

using namespace webotsQtUtils;

PoseEditor::PoseEditor(Pose *pose, Pose *previousPose, Pose *nextPose, bool fixedStep, QWidget *parent) :
  QDialog(parent),
  mPose(pose),
  mPreviousPose(previousPose),
  mNextPose(nextPose),
  mFixedStep(fixedStep) {
  assert(mPose);

  // create GUI
  setWindowTitle(tr("Pose editor"));

  QWidget *formWidget = new QWidget(this);
  mNameLineEdit = new QLineEdit(formWidget);
  mMinutesSpinBox = new QSpinBox(formWidget);
  mSecondsSpinBox = new QSpinBox(formWidget);
  mMilliSecondsSpinBox = new QSpinBox(formWidget);
  mMilliSecondsSpinBox->setRange(0, 999);
  QFormLayout *formLayout = new QFormLayout(formWidget);
  formLayout->addRow(tr("&Name:"), mNameLineEdit);
  formLayout->addRow(tr("&Minutes:"), mMinutesSpinBox);
  formLayout->addRow(tr("&Seconds:"), mSecondsSpinBox);
  formLayout->addRow(tr("&Milliseconds:"), mMilliSecondsSpinBox);

  QWidget *buttonsWidget = new QWidget(this);
  QPushButton *cancelButton = new QPushButton(tr("Cancel"), buttonsWidget);
  QPushButton *okButton = new QPushButton(tr("Ok"), buttonsWidget);
  QHBoxLayout *hBoxLayout = new QHBoxLayout(buttonsWidget);
  hBoxLayout->addStretch();
  hBoxLayout->addWidget(cancelButton);
  hBoxLayout->addWidget(okButton);

  QVBoxLayout *vBoxLayout = new QVBoxLayout(this);
  vBoxLayout->addWidget(formWidget);
  vBoxLayout->addWidget(buttonsWidget);

  // mPose -> GUI
  mNameLineEdit->setText(mPose->name());
  mMilliSecondsSpinBox->setValue(mPose->milliSeconds());
  mSecondsSpinBox->setValue(mPose->seconds());
  mMinutesSpinBox->setValue(mPose->minutes());

  // focus
  okButton->setFocus();

  // fixed step
  if (mFixedStep) {
    static const QString disableToolTipText(
      tr("The pose time cannot be set, because of the enabled fixed step parameter of the motion."));
    mMilliSecondsSpinBox->setEnabled(false);
    mMilliSecondsSpinBox->setToolTip(disableToolTipText);
    mSecondsSpinBox->setEnabled(false);
    mSecondsSpinBox->setToolTip(disableToolTipText);
    mMinutesSpinBox->setEnabled(false);
    mMinutesSpinBox->setToolTip(disableToolTipText);
  }

  // connection
  connect(okButton, SIGNAL(pressed()), this, SLOT(ok()));
  connect(cancelButton, SIGNAL(pressed()), this, SLOT(reject()));
  connect(mNameLineEdit, SIGNAL(returnPressed()), this, SLOT(ok()));
}

PoseEditor::~PoseEditor() {
}

void PoseEditor::ok() {
  if (mNameLineEdit->text().isEmpty()) {
    QMessageBox::warning(this, tr("Warning"), tr("The name cannot be empty"));
    mNameLineEdit->setFocus();
    return;
  }

  int time = mMilliSecondsSpinBox->value() + 1000 * mSecondsSpinBox->value() + 60000 * mMinutesSpinBox->value();

  if (!mFixedStep) {
    if (mPreviousPose && time <= mPreviousPose->time()) {
      QMessageBox::warning(
        this, tr("Warning"),
        tr("The new time cannot be lower or equal to the previous pose time (%1)").arg(mPreviousPose->toTimeString()));
      mMilliSecondsSpinBox->setFocus();
      return;
    } else if (mNextPose && time >= mNextPose->time()) {
      QMessageBox::warning(this, tr("Warning"),
                           tr("The new time cannot be bigger or equal the next pose time (%1)").arg(mNextPose->toTimeString()));
      mMilliSecondsSpinBox->setFocus();
      return;
    }
  }

  mPose->setTime(time);
  mPose->setName(mNameLineEdit->text());

  accept();
}
