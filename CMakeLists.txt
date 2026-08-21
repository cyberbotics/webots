cmake_minimum_required(VERSION 3.10)
project(webots)

include(GNUInstallDirs)

option(
  WEBOTS_PACKAGE_BUILD
  "Build bundled project controllers and libraries for inclusion in a binary package"
  OFF
)

# Handle dynamic loading library
if(WIN32)
    find_package(dlfcn-win32 REQUIRED)
    set(CMAKE_DL_LIBS dlfcn-win32::dl)
else()
    set(CMAKE_DL_LIBS ${CMAKE_DL_LIBS})
endif()

# Installation configuration
if(NOT CMAKE_INSTALL_PREFIX)
  # Check if we can write to /usr/local
  execute_process(COMMAND test -w /usr/local RESULT_VARIABLE can_write_usr_local)
  if(can_write_usr_local EQUAL 0)
    set(CMAKE_INSTALL_PREFIX "/usr/local" CACHE PATH "Installation prefix" FORCE)
  else()
    # Fallback to ~/.local
    set(CMAKE_INSTALL_PREFIX "$ENV{HOME}/.local" CACHE PATH "Installation prefix" FORCE)
  endif()
endif()

message(STATUS "Install prefix: ${CMAKE_INSTALL_PREFIX}")

if(WIN32)
  set(WEBOTS_DEV_EXECUTABLE_SUFFIX ".exe")
  if(MINGW)
    set(WEBOTS_DEV_SHARED_LIBRARY_PREFIX "lib")
  else()
    set(WEBOTS_DEV_SHARED_LIBRARY_PREFIX "")
  endif()
  set(WEBOTS_DEV_SHARED_LIBRARY_SUFFIX ".dll")
  set(WEBOTS_DEV_PATH_ENVIRONMENT_VARIABLE "PATH")
  set(WEBOTS_DEV_QT_QPA_PLATFORM "")
else()
  set(WEBOTS_DEV_EXECUTABLE_SUFFIX "")
  set(WEBOTS_DEV_SHARED_LIBRARY_PREFIX "lib")
  set(WEBOTS_DEV_SHARED_LIBRARY_SUFFIX ".so")
  set(WEBOTS_DEV_PATH_ENVIRONMENT_VARIABLE "LD_LIBRARY_PATH")
  set(WEBOTS_DEV_QT_QPA_PLATFORM "xcb")
endif()

if((UNIX AND NOT APPLE) OR WIN32)
  set(WEBOTS_DEV_BINARY_DIR "${CMAKE_BINARY_DIR}")
  set(WEBOTS_DEV_HOME "${CMAKE_SOURCE_DIR}")
  set(WEBOTS_DEV_EXECUTABLE "${CMAKE_BINARY_DIR}/src/webots/webots${WEBOTS_DEV_EXECUTABLE_SUFFIX}")
  set(WEBOTS_DEV_PKG_CONFIG_PATH "${CMAKE_BINARY_DIR}/pkgconfig")
  set(WEBOTS_DEV_CONTROLLER_LIB_PATH "${CMAKE_BINARY_DIR}/src/controller/c")
  set(WEBOTS_DEV_CONTROLLER_C_LIB
    "${CMAKE_BINARY_DIR}/src/controller/c/${WEBOTS_DEV_SHARED_LIBRARY_PREFIX}Controller${WEBOTS_DEV_SHARED_LIBRARY_SUFFIX}"
  )
  set(WEBOTS_DEV_CONTROLLER_CPP_LIB
    "${CMAKE_BINARY_DIR}/src/controller/cpp/${WEBOTS_DEV_SHARED_LIBRARY_PREFIX}CppController${WEBOTS_DEV_SHARED_LIBRARY_SUFFIX}"
  )
  set(WEBOTS_DEV_LIB_PATH "${CMAKE_BINARY_DIR}/src/ode")
  set(WEBOTS_DEV_PICOTTS_LIB_PATH "${CMAKE_BINARY_DIR}/external/picotts/lib")
  set(WEBOTS_DEV_CONTROLLER_C_LIB_PATH "${CMAKE_BINARY_DIR}/src/controller/c")
  set(WEBOTS_DEV_CONTROLLER_CPP_LIB_PATH "${CMAKE_BINARY_DIR}/src/controller/cpp")
  set(WEBOTS_DEV_WEBOTS_LIB_PATH "${CMAKE_BINARY_DIR}/src/webots")
  set(WEBOTS_DEV_CONTROLLER_PLUGINS_LIB_PATH "${CMAKE_BINARY_DIR}/controller_libs")

  if(WIN32)
    set(WEBOTS_DEV_CMD_PATH
      "${WEBOTS_DEV_CONTROLLER_C_LIB_PATH};${WEBOTS_DEV_CONTROLLER_CPP_LIB_PATH};${WEBOTS_DEV_CONTROLLER_PLUGINS_LIB_PATH};${WEBOTS_DEV_PICOTTS_LIB_PATH};${WEBOTS_DEV_LIB_PATH};${WEBOTS_DEV_WEBOTS_LIB_PATH}"
    )
  endif()

  configure_file(
    "${CMAKE_SOURCE_DIR}/scripts/webots-dev-env.sh.in"
    "${CMAKE_BINARY_DIR}/webots-dev-env.sh"
    @ONLY
  )
  configure_file(
    "${CMAKE_SOURCE_DIR}/scripts/webots-dev.in"
    "${CMAKE_BINARY_DIR}/webots-dev"
    @ONLY
  )
  if(WIN32)
    configure_file(
      "${CMAKE_SOURCE_DIR}/scripts/webots-dev.cmd.in"
      "${CMAKE_BINARY_DIR}/webots-dev.cmd"
      @ONLY
    )
  else()
    execute_process(COMMAND chmod +x "${CMAKE_BINARY_DIR}/webots-dev")
  endif()
endif()

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)
# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find Qt6 components
find_package(Qt6 REQUIRED COMPONENTS
    Core
    Gui
    OpenGL
    Qml
    QmlIntegration
    PrintSupport
    Widgets
    Xml
    OpenGLWidgets
    WebSockets
    Test
    Network
)
file(READ ${CMAKE_SOURCE_DIR}/resources/version.txt WEBOTS_VERSION)  # Path is correct - ${CMAKE_SOURCE_DIR} handles OS-specific path separators
string(STRIP ${WEBOTS_VERSION} WEBOTS_VERSION) 

file(MAKE_DIRECTORY "${CMAKE_BINARY_DIR}/pkgconfig" "${CMAKE_BINARY_DIR}/pkgconfig-install")
string(REPLACE "/" ";" WEBOTS_INSTALL_LIBDIR_PARTS "${CMAKE_INSTALL_LIBDIR}")
list(LENGTH WEBOTS_INSTALL_LIBDIR_PARTS WEBOTS_INSTALL_LIBDIR_DEPTH)
math(EXPR WEBOTS_PC_PREFIX_PARENT_COUNT "${WEBOTS_INSTALL_LIBDIR_DEPTH} + 1")
set(WEBOTS_PC_PREFIX_FROM_PCFILEDIR "")
foreach(_ RANGE 1 ${WEBOTS_PC_PREFIX_PARENT_COUNT})
  string(APPEND WEBOTS_PC_PREFIX_FROM_PCFILEDIR "/..")
endforeach()
set(WEBOTS_PC_PREFIX "\${pcfiledir}${WEBOTS_PC_PREFIX_FROM_PCFILEDIR}")
set(WEBOTS_PC_EXEC_PREFIX "\${prefix}")
set(WEBOTS_PC_LIBDIR "\${pcfiledir}/../webots")
set(WEBOTS_PC_INCLUDEDIR "\${prefix}/${CMAKE_INSTALL_INCLUDEDIR}/webots")
set(WEBOTS_PC_CONTROLLER_C_INCLUDEDIR "\${includedir}/controller/c")
set(WEBOTS_PC_CONTROLLER_CPP_INCLUDEDIR "\${includedir}/controller/cpp")
configure_file(
  "${CMAKE_SOURCE_DIR}/scripts/webots-controller.pc.in"
  "${CMAKE_BINARY_DIR}/pkgconfig-install/webots-controller.pc"
  @ONLY
)
configure_file(
  "${CMAKE_SOURCE_DIR}/scripts/webots-cpp-controller.pc.in"
  "${CMAKE_BINARY_DIR}/pkgconfig-install/webots-cpp-controller.pc"
  @ONLY
)
configure_file(
  "${CMAKE_SOURCE_DIR}/scripts/webots-ode.pc.in"
  "${CMAKE_BINARY_DIR}/pkgconfig-install/webots-ode.pc"
  @ONLY
)

set(WEBOTS_PC_PREFIX "${CMAKE_SOURCE_DIR}")
set(WEBOTS_PC_EXEC_PREFIX "${CMAKE_BINARY_DIR}")
set(WEBOTS_PC_LIBDIR "${CMAKE_BINARY_DIR}/src/controller/c")
set(WEBOTS_PC_INCLUDEDIR "${CMAKE_SOURCE_DIR}/include")
set(WEBOTS_PC_CONTROLLER_C_INCLUDEDIR "${CMAKE_SOURCE_DIR}/include/controller/c")
set(WEBOTS_PC_CONTROLLER_CPP_INCLUDEDIR "${CMAKE_SOURCE_DIR}/include/controller/cpp")
configure_file(
  "${CMAKE_SOURCE_DIR}/scripts/webots-controller.pc.in"
  "${CMAKE_BINARY_DIR}/pkgconfig/webots-controller.pc"
  @ONLY
)
set(WEBOTS_PC_LIBDIR "${CMAKE_BINARY_DIR}/src/controller/cpp")
configure_file(
  "${CMAKE_SOURCE_DIR}/scripts/webots-cpp-controller.pc.in"
  "${CMAKE_BINARY_DIR}/pkgconfig/webots-cpp-controller.pc"
  @ONLY
)
set(WEBOTS_PC_LIBDIR "${CMAKE_BINARY_DIR}/src/ode")
set(WEBOTS_PC_CONTROLLER_C_INCLUDEDIR "")
set(WEBOTS_PC_CONTROLLER_CPP_INCLUDEDIR "")
configure_file(
  "${CMAKE_SOURCE_DIR}/scripts/webots-ode.pc.in"
  "${CMAKE_BINARY_DIR}/pkgconfig/webots-ode.pc"
  @ONLY
)
# Find other required packages
find_package(OpenAL REQUIRED)
find_package(OpenGL REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(Freetype REQUIRED)
find_package(assimp REQUIRED)
pkg_check_modules(OPENVR REQUIRED IMPORTED_TARGET openvr)
# Include external dependencies
include(src/webots/cmake/ExternalDependencies.cmake)

# Add subdirectories
# Installation rules
install(DIRECTORY resources/
  DESTINATION ${CMAKE_INSTALL_DATADIR}/webots/resources
  USE_SOURCE_PERMISSIONS
  PATTERN ".gitkeep" EXCLUDE
  PATTERN "*.user" EXCLUDE
  PATTERN "*.lock" EXCLUDE
  PATTERN "build" EXCLUDE
  PATTERN "*.d" EXCLUDE
  PATTERN "*.o" EXCLUDE
)

install(DIRECTORY projects/
  DESTINATION ${CMAKE_INSTALL_DATADIR}/webots/projects
  USE_SOURCE_PERMISSIONS
  PATTERN ".gitkeep" EXCLUDE
  PATTERN "*.user" EXCLUDE
  PATTERN "*.lock" EXCLUDE
  # Keep editable sources and final controller binaries together, but do not
  # package compiler intermediates.  A relocated sample can therefore run
  # immediately and will create a fresh build directory when edited.
  PATTERN "build" EXCLUDE
  PATTERN "*.d" EXCLUDE
  PATTERN "*.o" EXCLUDE
)

install(DIRECTORY include/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/webots
)

# Add subdirectories
add_subdirectory(src)

# Install main executable
install(TARGETS webots
  RUNTIME DESTINATION bin
  BUNDLE DESTINATION .
)

# Install libraries
install(TARGETS Controller CppController
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}/webots
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}/webots
)

set(WEBOTS_SAMPLE_CONTROLLER_ENV
  WEBOTS_HOME=${CMAKE_SOURCE_DIR}
  WEBOTS_CONTROLLER_LIB_PATH=${CMAKE_BINARY_DIR}/controller_libs
  WEBOTS_LIB_PATH=${CMAKE_BINARY_DIR}/src/ode
  WEBOTS_CONTROLLER_C_LIB=${CMAKE_BINARY_DIR}/src/controller/c/libController.so
  WEBOTS_CONTROLLER_CPP_LIB=${CMAKE_BINARY_DIR}/src/controller/cpp/libCppController.so
)

if(WIN32)
  find_program(WEBOTS_BASH_EXECUTABLE bash REQUIRED)
  list(REMOVE_ITEM WEBOTS_SAMPLE_CONTROLLER_ENV
    WEBOTS_CONTROLLER_C_LIB=${CMAKE_BINARY_DIR}/src/controller/c/libController.so
    WEBOTS_CONTROLLER_CPP_LIB=${CMAKE_BINARY_DIR}/src/controller/cpp/libCppController.so
  )
  list(APPEND WEBOTS_SAMPLE_CONTROLLER_ENV
    WEBOTS_CONTROLLER_C_LIB=${CMAKE_BINARY_DIR}/src/controller/c/libController.dll.a
    WEBOTS_CONTROLLER_CPP_LIB=${CMAKE_BINARY_DIR}/src/controller/cpp/libCppController.dll.a
    "PKG_CONFIG_PATH=${CMAKE_BINARY_DIR}/pkgconfig:/ucrt64/lib/pkgconfig:/ucrt64/share/pkgconfig"
    WEBOTS_QT_VERSION=6
    WEBOTS_QT_INCLUDE_DIR=/ucrt64/include/qt6
    WEBOTS_QT_LIB_DIR=/ucrt64/lib
    "WEBOTS_QT_LIBRARIES=-lQt6Core -lQt6Gui -lQt6Widgets"
    "WEBOTS_QT_CORE_LIBRARIES=-lQt6Core"
    WEBOTS_QT_MOC=/ucrt64/bin/moc.exe
  )
  set(WEBOTS_BUILD_CONTROLLERS_TOOLCHAIN
    MSYSTEM=UCRT64
    PATH=/ucrt64/bin:/usr/bin
    CC=gcc
    CXX=g++
  )
  set(WEBOTS_BUILD_CONTROLLERS_COMMAND ${WEBOTS_BASH_EXECUTABLE} ${CMAKE_SOURCE_DIR}/scripts/build_controllers.sh)
else()
  list(APPEND WEBOTS_SAMPLE_CONTROLLER_ENV
    "PKG_CONFIG_PATH=${CMAKE_BINARY_DIR}/pkgconfig:$ENV{PKG_CONFIG_PATH}"
  )
  set(WEBOTS_BUILD_CONTROLLERS_TOOLCHAIN)
  set(WEBOTS_BUILD_CONTROLLERS_COMMAND ${CMAKE_SOURCE_DIR}/scripts/build_controllers.sh)
endif()

add_custom_target(webots_controller_sdk
  DEPENDS Controller CppController ode
  COMMENT "Building Webots controller SDK libraries"
)

add_custom_target(webots_sample_controllers
  COMMAND ${CMAKE_COMMAND} -E env
    ${WEBOTS_SAMPLE_CONTROLLER_ENV}
    ${WEBOTS_BUILD_CONTROLLERS_TOOLCHAIN}
    ${WEBOTS_BUILD_CONTROLLERS_COMMAND}
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
  COMMENT "Building Makefile-based controllers in projects and resources"
)
add_dependencies(webots_sample_controllers webots_controller_sdk)

set(WEBOTS_SAMPLE_CONTROLLERS_STRICT_ALL)
if(WEBOTS_PACKAGE_BUILD)
  set(WEBOTS_SAMPLE_CONTROLLERS_STRICT_ALL ALL)
endif()

add_custom_target(webots_sample_controllers_strict ${WEBOTS_SAMPLE_CONTROLLERS_STRICT_ALL}
  COMMAND ${CMAKE_COMMAND} -E env
    ${WEBOTS_SAMPLE_CONTROLLER_ENV}
    ${WEBOTS_BUILD_CONTROLLERS_TOOLCHAIN}
    WEBOTS_CONTROLLERS_STRICT=1
    ${WEBOTS_BUILD_CONTROLLERS_COMMAND}
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
  COMMENT "Building Makefile-based controllers in projects and resources with strict failure handling"
)
add_dependencies(webots_sample_controllers_strict webots_controller_sdk)

if(WEBOTS_PACKAGE_BUILD)
  message(STATUS "Package build: bundled project controllers and libraries will be built")
endif()

install(FILES
  "${CMAKE_BINARY_DIR}/pkgconfig-install/webots-controller.pc"
  "${CMAKE_BINARY_DIR}/pkgconfig-install/webots-cpp-controller.pc"
  "${CMAKE_BINARY_DIR}/pkgconfig-install/webots-ode.pc"
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/pkgconfig
)
