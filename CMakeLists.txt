cmake_minimum_required(VERSION 3.10)
project(webots)

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
  DESTINATION ${CMAKE_INSTALL_PREFIX}/share/webots/resources
  PATTERN ".gitkeep" EXCLUDE
  PATTERN "*.user" EXCLUDE
  PATTERN "*.lock" EXCLUDE
)

install(DIRECTORY projects/
  DESTINATION ${CMAKE_INSTALL_PREFIX}/share/webots/projects
  PATTERN ".gitkeep" EXCLUDE
  PATTERN "*.user" EXCLUDE
  PATTERN "*.lock" EXCLUDE
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
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)
