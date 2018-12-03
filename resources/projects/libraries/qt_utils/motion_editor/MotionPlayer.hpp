/*
 * Description:  Actuate the robot motors accorging to a motion
 */

#ifndef MOTION_PLAYER_HPP
#define MOTION_PLAYER_HPP

#include <QtCore/QObject>

namespace webotsQtUtils {
  class Motion;

  class MotionPlayer : public QObject {
    Q_OBJECT

  public:
    MotionPlayer();
    virtual ~MotionPlayer();

    void play(Motion *motion, int poseIndex, bool reverse);
    void stop();
    bool isPlaying() const { return static_cast<bool>(mMotion); }

    void writeActuators();

  signals:
    void motionEnded();

  public slots:
    void setLoop(bool loop) { mLoop = loop; }

  private:
    int getTime() const;  // ms
    void updateMotionDuration();

    Motion *mMotion;

    int mStartingTime;    // ms
    int mPoseTimeOffset;  // ms
    int mMotionDuration;  // ms
    bool mLoop;
    bool mReverse;
  };
}  // namespace webotsQtUtils

#endif
