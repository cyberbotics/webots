cmake_minimum_required(VERSION 3.10)
project(webots)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Set OpenGL preference to GLVND
set(OpenGL_GL_PREFERENCE GLVND)

# Find required packages
find_package(OpenGL REQUIRED)
find_package(Qt6 COMPONENTS Core Gui OpenGL REQUIRED)

# Add include directories globally
include_directories(
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/src/webots/external
)

# Define GLM experimental features
add_definitions(-DGLM_ENABLE_EXPERIMENTAL)

# Include cmake utilities
include(src/webots/cmake/FileUtils.cmake)

# Build glad as a library
add_library(glad STATIC ${CMAKE_SOURCE_DIR}/src/glad/glad.c)
target_include_directories(glad PUBLIC ${CMAKE_SOURCE_DIR}/src/webots/external)

# Add subdirectories
add_subdirectory(src)
