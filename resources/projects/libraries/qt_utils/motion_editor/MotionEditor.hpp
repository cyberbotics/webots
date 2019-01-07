/*
 * Description:  Motion editor main widget
 */

#ifndef MOTION_EDITOR_HPP
#define MOTION_EDITOR_HPP

#include <QtWidgets/QWidget>

class QCheckBox;
class QGroupBox;
class QMainWindow;
class QPushButton;
class QSpinBox;

namespace webotsQtUtils {

  class Motion;
  class MotionPlayer;
  class MotionWidget;

  class MotionEditor : public QWidget {
    Q_OBJECT

  public:
    explicit MotionEditor(QWidget *parent = NULL);
    virtual ~MotionEditor();

    void writeActuators();

  private slots:
    void newMotion();
    void openMotion();
    void saveMotion();
    void saveAsMotion();
    void play(bool playState);
    void reverse(bool playState);
    void pin(bool enable);
    void checkPlayAction();
    void updateFixedStep();
    void enableRunButtons();

  private:
    void play(bool playState, bool reverse);
    void createWidgetsAndLayouts();

    void initializeMotion(const QString &filename = "");

    Motion *mMotion;
    MotionPlayer *mMotionPlayer;
    MotionWidget *mMotionWidget;

    QGroupBox *mMotionFileOptions;
    QGroupBox *mInsertionOptions;

    QPushButton *mReverseButton;
    QPushButton *mPlayButton;
    QCheckBox *mLoopCheckBox;
    QCheckBox *mPinCheckBox;
    QCheckBox *mFixedStepCheckBox;
    QSpinBox *mFixedStepSpinBox;
  };
}  // namespace webotsQtUtils

#endif
