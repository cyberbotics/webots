/*
 * Description:  This class handles qt.
 *               It creates and refreshes the GUI, and listen to the input pipe
 *               of this process in order to return of the update function
 *               when required.
 *               - MacOS & Linux: a QSocketNotifier listen the pipe
 *               - Windows: the windows API allow to listen the pipe
 */

#ifndef MAIN_APPLICATION_HPP
#define MAIN_APPLICATION_HPP

#include <QtWidgets/QApplication>

#ifndef _WIN32
class QFile;
class QSocketNotifier;
#endif

namespace webotsQtUtils {

  class MainApplicationPrivate;

  class MainApplication {
  public:
    MainApplication();
    virtual ~MainApplication();

    virtual void preUpdateGui();
    virtual void updateGui();

    bool isInitialized() const;

  private:
    MainApplicationPrivate *mMainApplicationPrivate;
  };

  class MainApplicationPrivate : public QApplication {
    Q_OBJECT

  public:
    MainApplicationPrivate(int &argc, char **argv);
    virtual ~MainApplicationPrivate();

    virtual void preUpdateGui();
    virtual void updateGui();

    bool isInitialized() const { return mIsInitialized; }

#ifndef _WIN32
  private slots:
    void dataReadyOnPipeIn();
#endif

  private:
    bool mIsInitialized;
    bool mIsLeaving;
    size_t mPipeInHandler;

#ifdef _WIN32
    bool isMainWindowVisible() const;
#else
    QFile *mPipeInFile;
    QSocketNotifier *mSocketNotifier;
#endif
  };
}  // namespace webotsQtUtils

#endif
