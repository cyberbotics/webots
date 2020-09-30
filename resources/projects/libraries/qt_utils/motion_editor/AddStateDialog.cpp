#include "AddStateDialog.hpp"
#include "MotionGlobalSettings.hpp"

#include "Motion.hpp"
#include "MotorTargetState.hpp"
#include "Pose.hpp"

#include <devices/Motor.hpp>

#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QListWidget>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSpinBox>
#include <QtWidgets/QVBoxLayout>

#include <cassert>

using namespace webotsQtUtils;

AddStateDialog::AddStateDialog(Motion *motion, QWidget *parent) : QDialog(parent), mMotion(motion) {
  assert(mMotion);

  // create GUI
  setWindowTitle(tr("Add motor state"));

  QLabel *topLabel = new QLabel(this);
  topLabel->setText(tr("Choose one or more Motor(s) to add to this motion."));

  mListWidget = new QListWidget(this);
  populateListWidget();

  QWidget *buttonsWidget = new QWidget(this);
  QPushButton *cancelButton = new QPushButton(tr("Cancel"), buttonsWidget);
  QPushButton *okButton = new QPushButton(tr("Ok"), buttonsWidget);
  QHBoxLayout *hBoxLayout = new QHBoxLayout(buttonsWidget);
  hBoxLayout->addStretch();
  hBoxLayout->addWidget(cancelButton);
  hBoxLayout->addWidget(okButton);

  QVBoxLayout *vLayout = new QVBoxLayout(this);
  vLayout->addWidget(topLabel);
  vLayout->addWidget(mListWidget);
  vLayout->addWidget(buttonsWidget);

  // focus
  okButton->setFocus();

  // connection
  connect(okButton, SIGNAL(pressed()), this, SLOT(ok()));
  connect(cancelButton, SIGNAL(pressed()), this, SLOT(reject()));
}

AddStateDialog::~AddStateDialog() {
}

void AddStateDialog::populateListWidget() {
  assert(mListWidget);
  mListWidget->clear();

  QSet<QString> availableMotors;
  foreach (Motor *motor, MotionGlobalSettings::availableMotorList())
    availableMotors.insert(motor->name());

  QSet<QString> motorAlreadyDefined;
  if (mMotion->poses().count() > 0) {
    Pose *pose = mMotion->poses()[0];
    foreach (MotorTargetState *state, pose->states())
      motorAlreadyDefined.insert(state->motor()->name());
  }

  QSet<QString> motorsSet = availableMotors - motorAlreadyDefined;
  QStringList motors = motorsSet.values();
  motors.removeDuplicates();
  motors.sort();

  foreach (const QString &motor, motors) {
    mListWidget->addItem(motor);
    QListWidgetItem *insertedItem = mListWidget->item(mListWidget->count() - 1);
    insertedItem->setFlags(insertedItem->flags() | Qt::ItemIsUserCheckable);
    insertedItem->setCheckState(Qt::Unchecked);
  }
}

void AddStateDialog::ok() {
  for (int index = 0; index < mListWidget->count(); index++) {
    QListWidgetItem *item = mListWidget->item(index);
    if (item->checkState() == Qt::Checked)
      mMotion->addNewMotor(item->text());
  }

  accept();
}
