#include "MotionEditor.hpp"

#include "ListWidget.hpp"
#include "Motion.hpp"
#include "MotionPlayer.hpp"
#include "MotionWidget.hpp"

#include <devices/Device.hpp>
#include "core/StandardPaths.hpp"

#include <webots/robot.h>

#include <QtCore/QFileInfo>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QFileDialog>
#include <QtWidgets/QMessageBox>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSpinBox>
#include <QtWidgets/QVBoxLayout>

using namespace webotsQtUtils;

MotionEditor::MotionEditor(QWidget *parent) : QWidget(parent), mMotion(NULL) {
  mMotionPlayer = new MotionPlayer;
  connect(mMotionPlayer, SIGNAL(motionEnded()), this, SLOT(checkPlayAction()));

  createWidgetsAndLayouts();

  initializeMotion();

  setAutoFillBackground(true);
}

MotionEditor::~MotionEditor() {
  delete mMotionPlayer;
}

void MotionEditor::writeActuators() {
  mMotionPlayer->writeActuators();
}

void MotionEditor::createWidgetsAndLayouts() {
  mMotionWidget = new MotionWidget(this);

  QWidget *upperWidget = new QWidget(this);
  mMotionFileOptions = new QGroupBox(tr("Motion file options"), upperWidget);
  mMotionFileOptions->setObjectName("borderedGroupBox");
  QGroupBox *playOptions = new QGroupBox(tr("Play options"), upperWidget);
  playOptions->setObjectName("borderedGroupBox");
  mInsertionOptions = new QGroupBox(tr("Pose insertion options"), upperWidget);
  mInsertionOptions->setObjectName("borderedGroupBox");

  QPushButton *newButton = new QPushButton(mMotionFileOptions);
  newButton->setIcon(QIcon("icons:new_button.png"));
  newButton->setToolTip(tr("New motion..."));
  newButton->setObjectName("menuButton");
  connect(newButton, SIGNAL(pressed()), this, SLOT(newMotion()));

  QPushButton *openButton = new QPushButton(mMotionFileOptions);
  openButton->setIcon(QIcon("icons:open_button.png"));
  openButton->setToolTip(tr("Open motion..."));
  openButton->setObjectName("menuButton");
  connect(openButton, SIGNAL(pressed()), this, SLOT(openMotion()));

  QPushButton *saveButton = new QPushButton(mMotionFileOptions);
  saveButton->setIcon(QIcon("icons:save_button.png"));
  saveButton->setToolTip(tr("Save motion..."));
  saveButton->setObjectName("menuButton");
  connect(saveButton, SIGNAL(pressed()), this, SLOT(saveMotion()));

  QPushButton *saveAsButton = new QPushButton(mMotionFileOptions);
  saveAsButton->setIcon(QIcon("icons:save_as_button.png"));
  saveAsButton->setToolTip(tr("Save motion as..."));
  saveAsButton->setObjectName("menuButton");
  connect(saveAsButton, SIGNAL(pressed()), this, SLOT(saveAsMotion()));

  QHBoxLayout *motionOptionLayout = new QHBoxLayout(mMotionFileOptions);
  motionOptionLayout->addWidget(newButton);
  motionOptionLayout->addWidget(openButton);
  motionOptionLayout->addWidget(saveButton);
  motionOptionLayout->addWidget(saveAsButton);

  mReverseButton = new QPushButton(playOptions);
  mReverseButton->setIcon(QIcon("icons:reverse_button.png"));
  mReverseButton->setToolTip(tr("Play back motion"));
  mReverseButton->setCheckable(true);
  mReverseButton->setObjectName("menuButton");
  connect(mReverseButton, SIGNAL(toggled(bool)), this, SLOT(reverse(bool)));

  mPlayButton = new QPushButton(playOptions);
  mPlayButton->setIcon(QIcon("icons:real_time_button.png"));
  mPlayButton->setToolTip(tr("Play motion"));
  mPlayButton->setCheckable(true);
  mPlayButton->setObjectName("menuButton");
  connect(mPlayButton, SIGNAL(toggled(bool)), this, SLOT(play(bool)));

  mLoopCheckBox = new QCheckBox(tr("Loop"), playOptions);
  mLoopCheckBox->setToolTip(tr("Restart the motion directly when it ends"));
  connect(mLoopCheckBox, SIGNAL(toggled(bool)), mMotionPlayer, SLOT(setLoop(bool)));

  mPinCheckBox = new QCheckBox(tr("Pin"), playOptions);
  mPinCheckBox->setToolTip(tr("Attach the robot to the static environment"));
  connect(mPinCheckBox, SIGNAL(toggled(bool)), this, SLOT(pin(bool)));

  QHBoxLayout *playOptionLayout = new QHBoxLayout(playOptions);
  playOptionLayout->addWidget(mReverseButton);
  playOptionLayout->addWidget(mPlayButton);
  playOptionLayout->addWidget(mLoopCheckBox);
  playOptionLayout->addWidget(mPinCheckBox);

  mFixedStepCheckBox = new QCheckBox(tr("Fixed step"), mInsertionOptions);
  mFixedStepCheckBox->setToolTip(tr("Specify the constancy of the time gap between poses"));
  mFixedStepCheckBox->setChecked(false);
  connect(mFixedStepCheckBox, SIGNAL(toggled(bool)), this, SLOT(updateFixedStep()));

  mFixedStepSpinBox = new QSpinBox(mInsertionOptions);
  mFixedStepSpinBox->setToolTip(tr("Specify the time gap between consecutive poses"));
  mFixedStepSpinBox->setValue(wb_robot_get_basic_time_step());
  mFixedStepSpinBox->setRange(1, 1000);
  mFixedStepSpinBox->setSuffix(" " + tr("ms"));
  connect(mFixedStepSpinBox, SIGNAL(valueChanged(int)), this, SLOT(updateFixedStep()));

  QHBoxLayout *insertionOptionLayout = new QHBoxLayout(mInsertionOptions);
  insertionOptionLayout->addWidget(mFixedStepCheckBox);
  insertionOptionLayout->addWidget(mFixedStepSpinBox);

  QHBoxLayout *hBoxLayout = new QHBoxLayout(upperWidget);
  hBoxLayout->addWidget(mMotionFileOptions);
  hBoxLayout->addWidget(playOptions);
  hBoxLayout->addWidget(mInsertionOptions);
  hBoxLayout->addStretch();

  QVBoxLayout *vBoxLayout = new QVBoxLayout(this);
  vBoxLayout->addWidget(upperWidget);
  vBoxLayout->addWidget(mMotionWidget);
}

void MotionEditor::initializeMotion(const QString &filename) {
  delete mMotion;
  mMotion = new Motion(mMotionPlayer, filename);
  mMotionWidget->setMotion(mMotion);

  bool hasFixedStep = mMotion->hasFixedStep();
  mFixedStepCheckBox->blockSignals(true);
  mFixedStepCheckBox->setChecked(hasFixedStep);
  mFixedStepCheckBox->blockSignals(false);

  mFixedStepSpinBox->blockSignals(true);
  mFixedStepSpinBox->setValue(hasFixedStep ? mMotion->fixedStep() : wb_robot_get_basic_time_step());
  mFixedStepSpinBox->blockSignals(false);

  if (mMotion->hasInvalidMotorPositions())
    QMessageBox::warning(
      this, tr("Invalid Motor Positions"),
      tr("<p>This motion file contains some motor position values that lie outside the valid range.<br/>"
         "The wrong values and the corresponding poses are marked in <font color='#ff0000'>red</fonr>.</p>"));

  // disable buttons until a pose is selected
  mLoopCheckBox->setEnabled(false);
  mPinCheckBox->setEnabled(false);
  mReverseButton->setEnabled(false);
  mPlayButton->setEnabled(false);
  connect(mMotionWidget, SIGNAL(selectedPoseChanged()), this, SLOT(enableRunButtons()));
}

void MotionEditor::enableRunButtons() {
  disconnect(mMotionWidget, SIGNAL(selectedPoseChanged()), this, SLOT(enableRunButtons()));
  mLoopCheckBox->setEnabled(true);
  mPinCheckBox->setEnabled(true);
  mReverseButton->setEnabled(true);
  mPlayButton->setEnabled(true);
}

void MotionEditor::newMotion() {
  initializeMotion();
}

void MotionEditor::openMotion() {
  static QString lastDirectory = StandardPaths::getProjectPath();
  QString filePath =
    QFileDialog::getOpenFileName(this, tr("Open motion file"), lastDirectory, tr("Webots Motion Files (*.motion)"), NULL);
  if (filePath.isEmpty())  // the user pressed cancel
    return;

  initializeMotion(filePath);
  lastDirectory = QFileInfo(filePath).absolutePath();
}

void MotionEditor::saveMotion() {
  if (mMotion->isDefaultFilePath()) {
    saveAsMotion();
    return;
  }

  mMotion->save();
}

void MotionEditor::saveAsMotion() {
  QString filePath =
    QFileDialog::getSaveFileName(this, tr("Save motion file"), mMotion->filePath(), tr("Webots Motion Files (*.motion)"), NULL);

  if (filePath.isEmpty())
    return;

  mMotion->setFilePath(filePath, false);
  mMotion->save();
}

void MotionEditor::checkPlayAction() {
  if (mPlayButton->isChecked())
    mPlayButton->toggle();
  else if (mReverseButton->isChecked())
    mReverseButton->toggle();
}

void MotionEditor::updateFixedStep() {
  if (mFixedStepCheckBox->isChecked()) {
    int step = mFixedStepSpinBox->value();

    if (mMotion->hasFixedStep())
      mMotion->setFixedStep(step);
    else {
      QMessageBox::StandardButton answer =
        QMessageBox::question(this, tr("Set fixed step"),
                              tr("This operation will set all the time gap between poses to %1 [ms]. "
                                 "Some of the modified data will be lost after this operation. "
                                 "Would you like to procees anyway?")
                                .arg(step),
                              QMessageBox::Ok | QMessageBox::Cancel, QMessageBox::Cancel);

      if (answer == QMessageBox::Ok)
        mMotion->setFixedStep(step);
      else {
        mFixedStepCheckBox->blockSignals(true);
        mFixedStepCheckBox->setChecked(false);
        mFixedStepCheckBox->blockSignals(false);
      }
    }
  } else
    mMotion->setFixedStep(0);
}

void MotionEditor::play(bool playState, bool reverse) {
  static const QIcon playIcon(QIcon("icons:real_time_button.png"));
  static const QIcon reverseIcon(QIcon("icons:reverse_button.png"));
  static const QIcon stopIcon(QIcon("icons:pause_button.png"));

  bool enable = !playState;

  QPushButton *pressedButton = NULL;
  QPushButton *dualButton = NULL;
  if (reverse) {
    pressedButton = mReverseButton;
    dualButton = mPlayButton;
  } else {
    pressedButton = mPlayButton;
    dualButton = mReverseButton;
  }

  mMotionWidget->setEnabled(enable);
  mMotionFileOptions->setEnabled(enable);
  mInsertionOptions->setEnabled(enable);
  mLoopCheckBox->setEnabled(enable);
  mPinCheckBox->setEnabled(enable);
  dualButton->setEnabled(enable);

  if (playState) {
    pressedButton->setIcon(stopIcon);
    mMotionPlayer->play(mMotion, mMotionWidget->currentIndex(), reverse);
  } else {
    if (reverse)
      pressedButton->setIcon(reverseIcon);
    else
      pressedButton->setIcon(playIcon);
    mMotionPlayer->stop();
  }
}

void MotionEditor::play(bool playState) {
  play(playState, false);
}

void MotionEditor::reverse(bool playState) {
  play(playState, true);
}

void MotionEditor::pin(bool enable) {
  wb_robot_pin_to_static_environment(enable);
}
