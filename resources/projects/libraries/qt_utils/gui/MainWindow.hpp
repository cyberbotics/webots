/*
 * Description:  Abstract class defining a generic MainWindow
 */

#ifndef MAIN_WINDOW_HPP
#define MAIN_WINDOW_HPP

#include <QtWidgets/QMainWindow>

class QCloseEvent;

namespace webotsQtUtils {

  class MainWindow : public QMainWindow {
  public:
    MainWindow();
    virtual ~MainWindow();

    virtual void showWindow();

  protected:
    virtual void closeEvent(QCloseEvent *event);

#ifdef _WIN32
    virtual void hideEvent(QHideEvent *event);
    virtual void showEvent(QShowEvent *event);

  private:
    void processApplicationEvents();
#endif
  };
}  // namespace webotsQtUtils

#endif
