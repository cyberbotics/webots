# src/controller/cpp/CMakeLists.txt

# Name of the C++ controller library
set(CONTROLLER_CPP_LIBRARY_NAME CppController)

# Source files for the C++ controller library
set(CONTROLLER_CPP_SOURCES
  Accelerometer.cpp
  Altimeter.cpp
  Brake.cpp
  Camera.cpp
  Compass.cpp
  Connector.cpp
  Device.cpp
  Display.cpp
  DistanceSensor.cpp
  Emitter.cpp
  Field.cpp
  GPS.cpp
  Gyro.cpp
  InertialUnit.cpp
  Joystick.cpp
  Keyboard.cpp
  LED.cpp
  Lidar.cpp
  LightSensor.cpp
  Motion.cpp
  Motor.cpp
  Mouse.cpp
  Node.cpp
  Pen.cpp
  PositionSensor.cpp
  Proto.cpp
  Radar.cpp
  RangeFinder.cpp
  Receiver.cpp
  Robot.cpp
  Skin.cpp
  Speaker.cpp
  Supervisor.cpp
  TouchSensor.cpp
  VacuumGripper.cpp
)

# Create the shared library
add_library(${CONTROLLER_CPP_LIBRARY_NAME} SHARED ${CONTROLLER_CPP_SOURCES})

# Include directories for the C++ controller library
target_include_directories(${CONTROLLER_CPP_LIBRARY_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:include/controller/cpp>
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include> # added because some include files are not in the cpp folder
    $<INSTALL_INTERFACE:include> # added because some include files are not in the cpp folder#
  PRIVATE
  $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include/controller/c>  
  $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include/controller/cpp>  
)
target_compile_definitions(${CONTROLLER_CPP_LIBRARY_NAME}
  PUBLIC
    "LIBCONTROLLER_VERSION=\"${WEBOTS_VERSION}\""
)
# Link necessary libraries to the C++ controller
target_link_libraries(${CONTROLLER_CPP_LIBRARY_NAME}
  PUBLIC
  # glad # Assuming glad is linked to the C controller and might be needed here as well
  # ${Freetype_LIBRARIES} # Assuming Freetype is linked to the C controller and might be needed here as well
)

# Set properties for the library (adjust as needed)
#set_target_properties(${CONTROLLER_CPP_LIBRARY_NAME} PROPERTIES
#  VERSION 0.1
#  SOVERSION 0
#  PUBLIC_HEADER "${CONTROLLER_CPP_HEADERS}" # No need for PUBLIC_HEADER with C++
#)

# Install rules
install(TARGETS ${CONTROLLER_CPP_LIBRARY_NAME}
  DESTINATION lib
)

# No need to install header files separately in C++, they are typically included via target_include_directories
