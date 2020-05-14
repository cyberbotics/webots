/*
 * Description:  Motion global settings
 */

#ifndef MOTION_GLOBAL_SETTINGS_HPP
#define MOTION_GLOBAL_SETTINGS_HPP

#include <QtCore/QList>

namespace webotsQtUtils {

  class Motor;

  class MotionGlobalSettings {
  public:
    static void setAvailableMotorList(const QList<Motor *> &motors);
    static const QList<Motor *> &availableMotorList() { return cMotors; }

    static double precision() { return 3; }

  private:
    static QList<Motor *> cMotors;

    MotionGlobalSettings() {}
    ~MotionGlobalSettings() {}
  };
}  // namespace webotsQtUtils

#endif
