# Name of the library to be created
set(CONTROLLER_LIBRARY_NAME Controller)

# Source files for the controller library
set(CONTROLLER_SOURCES
  abstract_camera.c
  accelerometer.c
  altimeter.c
  ansi_codes.c
  base64.c
  brake.c
  camera.c
  compass.c
  connector.c
  console.c
  default_robot_window.c
  device.c
  display.c
  distance_sensor.c
  dynamic_library.c
  emitter.c
  file.c
  g_image.c
  g_pipe.c
  gps.c
  gyro.c
  html_robot_window.c
  image.c
  inertial_unit.c
  joystick.c
  keyboard.c
  led.c
  lidar.c
  light_sensor.c
  microphone.c
  motion.c
  motor.c
  mouse.c
  node.c
  pen.c
  percent.c
  position_sensor.c
  radar.c
  radio.c
  range_finder.c
  receiver.c
  remote_control.c
  request.c
  robot.c
  robot_window.c
  scheduler.c
  sha1.c
  skin.c
  speaker.c
  string.c
  supervisor.c
  system.c
  tcp_client.c
  touch_sensor.c
  vacuum_gripper.c
)

# Header files for internal use within the library
set(CONTROLLER_HEADERS
  abstract_camera.h
  base64.h
  camera_private.h
  default_robot_window_private.h
  device_private.h
  display_private.h
  dynamic_library.h
  file.h
  g_image.h
  g_pipe.h
  html_robot_window_private.h
  image_private.h
  joystick_private.h
  keyboard_private.h
  messages.h
  motion_private.h
  mouse_private.h
  percent.h
  remote_control_private.h
  request.h
  robot_private.h
  robot_window_private.h
  scheduler.h
  sha1.h
  supervisor_private.h
  tcp_client.h
)

# Create the shared library
add_library(${CONTROLLER_LIBRARY_NAME} SHARED ${CONTROLLER_SOURCES} ${CONTROLLER_HEADERS})

# Include directories for the controller library
 target_include_directories(${CONTROLLER_LIBRARY_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:include>
    $<BUILD_INTERFACE:${STB_INCLUDE_DIR}>
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include/controller/c>
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/controller/c>
)
# Find libraries on Windows (using MSYS2)
if (WIN32)
    # Directly use MSYSTEM_PREFIX for library paths
    set(winsock_LIBRARY_PATH "$ENV{MSYSTEM_PREFIX}/lib/libws2_32.a")

    # Check if winsock library exists and link it
    if (EXISTS ${winsock_LIBRARY_PATH})
        message(STATUS "winsock library found: ${winsock_LIBRARY_PATH}")
    else()
        message(FATAL_ERROR "winsock library not found at ${winsock_LIBRARY_PATH}. Please ensure it is installed in your MSYS2 environment.")
    endif()

    # Link libraries if found
    target_link_libraries(core_lib
        PRIVATE
        ${winsock_LIBRARY_PATH}
    )
endif()

target_compile_definitions(${CONTROLLER_LIBRARY_NAME}
  PUBLIC
    "LIBCONTROLLER_VERSION=\"${WEBOTS_VERSION}\""
)
# Link necessary libraries to the controller
target_link_libraries(${CONTROLLER_LIBRARY_NAME}
  PUBLIC
  #    glad
  #   ${Freetype_LIBRARIES}
)

# Set properties for the library (adjust as needed)
#set_target_properties(${CONTROLLER_LIBRARY_NAME} PROPERTIES
#  VERSION 0.1
#  SOVERSION 0
#  PUBLIC_HEADER "${CONTROLLER_HEADERS}"
#)

# Install rules
install(TARGETS ${CONTROLLER_LIBRARY_NAME}
  DESTINATION lib
)

install(FILES ${CONTROLLER_HEADERS}
  DESTINATION include
)
