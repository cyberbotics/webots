cmake_minimum_required(VERSION 3.10)
project(webots)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find Qt6
find_package(Qt6 COMPONENTS Core Gui OpenGL QmlIntegration REQUIRED)

# Include external dependencies
include(src/webots/cmake/ExternalDependencies.cmake)

# Add subdirectory
add_subdirectory(src)
