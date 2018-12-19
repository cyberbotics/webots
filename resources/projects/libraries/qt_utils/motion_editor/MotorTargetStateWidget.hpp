/*
 * Description:  motor target state view
 */

#ifndef MOTOR_TARGET_STATE_WIDGET_HPP
#define MOTOR_TARGET_STATE_WIDGET_HPP

#include <QtWidgets/QGroupBox>

class QCheckBox;
class QDoubleSpinBox;
class QLabel;
class QSlider;

namespace webotsQtUtils {

  class MotorTargetState;

  class MotorTargetStateWidget : public QGroupBox {
    Q_OBJECT

  public:
    explicit MotorTargetStateWidget(QWidget *parent = NULL);
    virtual ~MotorTargetStateWidget();

    void changeState(MotorTargetState *state);

  private:
    void updateRange();
    void updateTitleAndEnable();

    void valueToSlider(double value);
    double sliderToValue() const;

    MotorTargetState *mState;
    double mMinimum;
    double mMaximum;

    QCheckBox *mCheckBox;
    QDoubleSpinBox *mSpinBox;
    QLabel *mMinLabel;
    QLabel *mMaxLabel;
    QSlider *mSlider;

  private slots:
    void clearStatePointer();
    void updateStateFromModel();
    void checkBoxToModel(int check);
    void spinBoxToModel(double value);
    void sliderToModel(int value);
  };
}  // namespace webotsQtUtils

#endif
