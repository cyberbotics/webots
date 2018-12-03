/*
 * Description:  Class defining the standard paths
 */

#ifndef STANDARD_PATHS_HPP
#define STANDARD_PATHS_HPP

#include <QtCore/QString>

namespace webotsQtUtils {

  class StandardPaths {
  public:
    static const QString &getWebotsHomePath();
    static const QString &getCurrentLibraryPath();
    static const QString &getControllerPath();
    static const QString &getProjectPath();

  private:
    StandardPaths() {}
  };

}  // namespace webotsQtUtils

#endif
