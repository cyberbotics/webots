/*
 * Description:  Motion
 */

#ifndef MOTION_HPP
#define MOTION_HPP

#include <QtCore/QList>
#include <QtCore/QObject>

namespace webotsQtUtils {

  class Pose;
  class MotionPlayer;

  class Motion : public QObject {
    Q_OBJECT

  public:
    static Motion *instance();

    explicit Motion(const MotionPlayer *player, const QString &filepath = "");
    virtual ~Motion();

    const QList<Pose *> &poses() const { return mPoses; }
    const QString &name() const { return mName; }
    const QString &filePath() const { return mFilePath; }
    const bool isDefaultFilePath() const { return mDefaultFilePath; }

    void setFilePath(const QString &filepath, bool checkExistence);

    void save() const;

    void applyPoseToRobot(Pose *pose);

    bool hasFixedStep() const { return mFixedStep > 0; }
    int fixedStep() const { return mFixedStep; }
    void setFixedStep(int step);

    void newPoseAt(int index);
    void duplicatePoseAt(int index);
    void deletePoseAt(int index);
    void swapPoseWithNext(int index);

    bool isSomeStateDefinedByMotorName(const QString &motorName) const;
    void deleteStatesByMotorName(const QString &motorName);
    void addNewMotor(const QString &motorName);

    bool hasInvalidMotorPositions() const { return !mIsValid; }

    void blockPoseSelection(bool blocked) { mIsPoseSelectionBlocked = blocked; }
    bool isPoseSelectionBlocked() const { return mIsPoseSelectionBlocked; }
    int selectedPoseIndex() const { return mSelectedPoseIndex; }

  signals:
    void updated();
    void poseSelected(int index);
    void poseUpdated(int index);
    void poseInserted(int index);
    void poseDeleted(int index);

  private:
    int computePoseToIndex(Pose *p) const;
    bool estimateHasFixedStep() const;
    int estimateFixedStepValue() const;

    bool validate() const;
    void load();
    void parse();
    void clear();

    static Motion *cInstance;

    int mFixedStep;
    int mNewPoseCounter;
    bool mDefaultFilePath;
    const MotionPlayer *mPlayer;
    QString mName;
    QString mFilePath;
    QList<Pose *> mPoses;

    bool mIsValid;
    bool mIsPoseSelectionBlocked;
    int mSelectedPoseIndex;

  private slots:
    void propagatePoseUpdate();
    void propagateSelection();
  };
}  // namespace webotsQtUtils

#endif
