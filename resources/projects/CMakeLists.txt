# Placeholder CMakeLists.txt for resources/projects
# TODO: Convert the logic from resources/projects/Makefile to CMake.
# This will involve adding subdirectories for controllers, libraries, plugins, worlds.

cmake_minimum_required(VERSION 3.10)
project(webots_resources_projects)

message(STATUS "Configuring resources/projects")

# Add subdirectories. Each of these will require its own CMakeLists.txt.
add_subdirectory(controllers)
add_subdirectory(libraries)
add_subdirectory(plugins)
add_subdirectory(worlds)

# Add build targets, sources, and installation rules here as needed
# for any direct content of resources/projects, if any.