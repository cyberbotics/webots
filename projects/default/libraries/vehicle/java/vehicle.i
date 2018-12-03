%module vehicle

%pragma(java) jniclasscode=%{
  static {
    try {
      System.loadLibrary("vehicle");
    } catch (UnsatisfiedLinkError e) {
      System.err.println("Native code library failed to load. See the chapter on Dynamic Linking Problems in the SWIG Java documentation for help.\n" + e);
      System.exit(1);
    }
  }
%}

%include "enumsimple.swg"
%javaconst(1);

%{
#include <webots/vehicle/Driver.hpp>
#include <webots/vehicle/Car.hpp>
%}

%import "controller.i"

%typemap(javaimports) SWIGTYPE %{
import com.cyberbotics.webots.controller.Supervisor;
%}

%include <webots/vehicle/Driver.hpp>
%include <webots/vehicle/Car.hpp>
