%module managers

%{
#include <webots/DifferentialWheels.hpp> // avoid '‘DifferentialWheels’ is not a member of ‘webots’' error
#include <webots/Supervisor.hpp>         // avoid '‘Supervisor’ is not a member of ‘webots’' error  
#include <RobotisOp2GaitManager.hpp>
#include <RobotisOp2MotionManager.hpp>
%}

%import "controller.i"

%include <RobotisOp2GaitManager.hpp>
%include <RobotisOp2MotionManager.hpp>
