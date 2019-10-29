/*
 * Description:  Target state of a Motor
 */

#ifndef MOTOR_TARGET_STATE_HPP
#define MOTOR_TARGET_STATE_HPP

#include <QtCore/QObject>

namespace webotsQtUtils {
  class Motor;

  class MotorTargetState : public QObject {
    Q_OBJECT

  public:
    enum Status { NORMAL, DISABLED, MODIFIED, INVALID };

    explicit MotorTargetState(Motor *motor);
    explicit MotorTargetState(const MotorTargetState &other);
    virtual ~MotorTargetState();

    QString toString() const;
    Status status() const;

    // getters
    Motor *motor() const { return mMotor; }
    double value() const { return mCurrentValue; }
    double minValue() const { return mMinValue; }
    double maxValue() const { return mMaxValue; }
    bool isRangeUnlimited() const { return mIsRangeUnlimited; }
    bool isDefined() const { return mCurrentDefined; }
    bool isValid() const { return mIsValid; }
    bool isModified() const { return mIsModified; }

    // setters
    void setValue(double value);
    void setDefined(bool defined);
    void reset();

    // setters called from the parser, returns if the stored value is valid
    bool setValueFromParser(const QString &value);

    // update stored value after saving
    void updateAfterSaving();

    static double defaultMinValue();
    static double defaultMaxValue();

  signals:
    void updated();
    void validChanged();
    void modifiedChanged(bool modified);

  private:
    MotorTargetState &operator=(const MotorTargetState &);  // non copyable
    static double precisionReducer(double value);
    void updateIsModified();
    void updateIsValid();
    void updateRange();

    Motor *mMotor;
    double mSavedValue;
    double mCurrentValue;
    bool mSavedDefined;
    bool mCurrentDefined;

    bool mIsValid;
    bool mIsModified;

    bool mIsRangeUnlimited;
    double mMinValue;
    double mMaxValue;
  };
}  // namespace webotsQtUtils

#endif
