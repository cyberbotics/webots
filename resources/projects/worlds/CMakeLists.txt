# Placeholder CMakeLists.txt for resources/projects/worlds
# TODO: Convert the logic from the corresponding Makefile section to CMake.

cmake_minimum_required(VERSION 3.10)
project(webots_resources_projects_worlds)

message(STATUS "Configuring resources/projects/worlds")

# Worlds are typically assets. This file might be used for:
# - Custom commands to process world files.
# - Installing world files to the correct location.
# Example:
# install(DIRECTORY . DESTINATION share/webots/projects/worlds FILES_MATCHING PATTERN "*.wbt")