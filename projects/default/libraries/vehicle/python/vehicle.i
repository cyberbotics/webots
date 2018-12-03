%module vehicle

%{
#include <webots/DifferentialWheels.hpp> // avoid '‘DifferentialWheels’ is not a member of ‘webots’' error
#include <webots/vehicle/Driver.hpp>
#include <webots/vehicle/Car.hpp>
%}

%import "controller.i"

%include <webots/vehicle/Driver.hpp>
%include <webots/vehicle/Car.hpp>
