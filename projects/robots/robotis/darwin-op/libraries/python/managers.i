%module managers

%pythonbegin %{
import sys
import os
if os.name == 'nt' and sys.version_info >= (3, 8):  # we need to explicitly list the folders containing the DLLs
    robotis_libraries = os.path.join(os.environ['WEBOTS_HOME'], 'projects', 'robots', 'robotis', 'darwin-op', 'libraries')
    os.add_dll_directory(os.path.join(robotis_libraries, 'managers'))
    os.add_dll_directory(os.path.join(robotis_libraries, 'robotis-op2'))
%}

%{
#include <webots/Supervisor.hpp>         // avoid '‘Supervisor’ is not a member of ‘webots’' error
#include <RobotisOp2GaitManager.hpp>
#include <RobotisOp2MotionManager.hpp>
%}

%import "controller.i"

%include <RobotisOp2GaitManager.hpp>
%include <RobotisOp2MotionManager.hpp>
