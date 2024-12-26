cmake_minimum_required(VERSION 3.10)
project(webots)
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
add_subdirectory(src)
