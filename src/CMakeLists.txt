cmake_minimum_required(VERSION 3.5)
project(webots-project)

# Set C++ standard globally
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find common dependencies
find_package(OpenGL REQUIRED)
find_package(Qt6 COMPONENTS Core Gui OpenGL REQUIRED)

# Define GLM experimental features
add_definitions(-DGLM_ENABLE_EXPERIMENTAL)

# Include cmake utilities
include(webots/cmake/FileUtils.cmake)

# Add subdirectories
add_subdirectory(wren)
add_subdirectory(webots)
