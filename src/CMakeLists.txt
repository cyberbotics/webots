# Find required packages

# Add glad library
add_library(glad glad/glad.c)
target_include_directories(glad PUBLIC ${CMAKE_SOURCE_DIR}/include)
target_compile_options(glad PUBLIC -fPIC)

add_custom_target(stb
  COMMAND git submodule update --init --recursive src/stb
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
  COMMENT "Initializing stb submodule"
)

# --- Handle stb_image.h ---

# Check for system installation of stb
find_path(STB_INCLUDE_DIR
  NAMES stb_image.h
  PATHS
    /usr/include/stb
    /usr/local/include/stb
    $ENV{MSYSTEM_PREFIX}/include/stb
    $ENV{MSYSTEM_PREFIX}/local/include/stb
)

# If not found, use the stb submodule
if(NOT STB_INCLUDE_DIR)
  message(STATUS "System installation of stb not found. Using stb submodule.")

  # Initialize and update the stb submodule
  execute_process(
    COMMAND ${CMAKE_COMMAND} --build . --target stb
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
  )

  # Set STB_INCLUDE_DIR to the submodule's include path
  set(STB_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/stb)

  # Cache the variable so it persists across CMake runs
  set(STB_INCLUDE_DIR ${STB_INCLUDE_DIR} CACHE PATH "Path to stb include directory")
endif()

message(STATUS "STB_INCLUDE_DIR: ${STB_INCLUDE_DIR}")

# Make STB_INCLUDE_DIR available to other CMake files (e.g., in src/controller/c)
set(STB_INCLUDE_DIR ${STB_INCLUDE_DIR} CACHE PATH "Path to stb include directory" FORCE)

add_subdirectory(ode)
add_subdirectory(wren)
add_subdirectory(webots)
add_subdirectory(controller)
