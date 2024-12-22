cmake_minimum_required(VERSION 3.1)
project(webots)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find required packages
find_package(OpenGL REQUIRED)
find_package(Qt6 COMPONENTS Core Gui OpenGL REQUIRED)

# Add include directories globally
include_directories(${CMAKE_SOURCE_DIR}/include)

# Add subdirectories
add_subdirectory(src/wren)
