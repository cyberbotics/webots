/*
 * Description:  Pose
 */

#ifndef POSE_HPP
#define POSE_HPP

#include <QtCore/QObject>

namespace webotsQtUtils {

  class Motor;
  class MotorTargetState;

  class Pose : public QObject {
    Q_OBJECT

  public:
    enum Status { NORMAL, MODIFIED, INVALID };

    Pose();
    Pose(const Pose &other);
    virtual ~Pose();

    QString toString() const;
    QString toTimeString() const;
    Status status() const;

    void createAndAppendState(Motor *motor);
    void deleteStateAt(int index);
    void swapStateWithNext(int index);

    MotorTargetState *findStateByMotorName(const QString &motorName) const;
    int computeIndexOfStateByMotorName(const QString &motorName) const;

    // getters
    const QString &name() const { return mName; }
    int time() const { return mTime; }  // ms
    int milliSeconds() const;
    int seconds() const;
    int minutes() const;
    const QList<MotorTargetState *> &states() const { return mStates; }

    // setters called from the parser
    void setTimeFromParser(const QString &time);
    void setNameFromParser(const QString &name);
    // read value and return if it is valid
    bool setMotorValueFromParser(Motor *motor, const QString &value);

    // regular setters
    void select() { emit selected(); }
    void setName(const QString &name);
    void setTime(int time);
    void addNewMotor(Motor *motor);

  signals:
    void selected();
    void updated(bool isStatusUpdate = false);
    void stateUpdated(int index);
    void stateInserted(int index);
    void stateDeleted(int index);

  private:
    Pose &operator=(const Pose &);  // non copyable
    MotorTargetState *createOrRetrieve(Motor *motor);
    void clear();
    int computeStateToIndex(MotorTargetState *s) const;

    QString mName;
    int mTime;  // ms

    QList<MotorTargetState *> mStates;
    bool mIsValid;
    bool mIsModified;

  private slots:
    void propagateStateUpdate();
    void updateIsModified(bool modified);
    void updateIsValid();
  };
}  // namespace webotsQtUtils

#endif
