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

# find_path() trusts cached values without checking whether they still exist.
# Drop a stale value so switching machines or removing the system package does
# not produce a successful configuration followed by a compiler failure.
if(DEFINED STB_INCLUDE_DIR AND NOT EXISTS "${STB_INCLUDE_DIR}/stb_image.h")
  unset(STB_INCLUDE_DIR CACHE)
  unset(STB_INCLUDE_DIR)
endif()

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

  # Initialize the bundled headers directly. Invoking a CMake build target
  # here is invalid because this source tree is still being configured.
  find_package(Git REQUIRED)
  execute_process(
    COMMAND ${GIT_EXECUTABLE} submodule update --init --recursive src/stb
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    RESULT_VARIABLE STB_SUBMODULE_RESULT
  )

  if(NOT STB_SUBMODULE_RESULT EQUAL 0)
    message(FATAL_ERROR "Failed to initialize the stb submodule")
  endif()

  # Set STB_INCLUDE_DIR to the submodule's include path
  set(STB_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/src/stb)

  # Cache the variable so it persists across CMake runs
  set(STB_INCLUDE_DIR ${STB_INCLUDE_DIR} CACHE PATH "Path to stb include directory")
endif()

if(NOT EXISTS "${STB_INCLUDE_DIR}/stb_image.h")
  message(FATAL_ERROR "STB_INCLUDE_DIR does not contain stb_image.h: ${STB_INCLUDE_DIR}")
endif()

message(STATUS "STB_INCLUDE_DIR: ${STB_INCLUDE_DIR}")

# Make STB_INCLUDE_DIR available to other CMake files (e.g., in src/controller/c)
set(STB_INCLUDE_DIR ${STB_INCLUDE_DIR} CACHE PATH "Path to stb include directory" FORCE)

add_subdirectory(ode)
add_subdirectory(wren)
add_subdirectory(webots)
add_subdirectory(controller)
