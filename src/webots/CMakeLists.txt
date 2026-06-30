cmake_minimum_required(VERSION 3.10)
project(Webots VERSION 1.0)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Enable PIC globally
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Enable RTTI globally
add_compile_options(-frtti)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -frtti")
if(WIN32)
    add_definitions(-DUNICODE -D_UNICODE)
endif()
# Set visibility settings globally
add_compile_options(-fvisibility=default)
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    add_compile_options(-fno-gnu-unique)
endif()

# Add cmake modules
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
include(FileUtils)


# Find OIS
find_package(PkgConfig REQUIRED)
pkg_check_modules(OIS REQUIRED IMPORTED_TARGET OIS)


# Add subdirectories
add_subdirectory(app)
add_subdirectory(control)
add_subdirectory(core)
add_subdirectory(editor)
add_subdirectory(engine)
add_subdirectory(gui)
add_subdirectory(maths)
add_subdirectory(nodes)
add_subdirectory(ode)
add_subdirectory(plugins)
add_subdirectory(scene_tree)
add_subdirectory(sound)
add_subdirectory(user_commands)
add_subdirectory(util)
add_subdirectory(vrml)
add_subdirectory(widgets)
add_subdirectory(wren)

# Main executable
add_executable(webots gui/main.cpp)
if(MINGW) # Or if you want it for other compilers, adjust the condition
    target_link_options(webots PRIVATE "-Wl,-Map,${CMAKE_CURRENT_BINARY_DIR}/webots.map")
    # For MSVC, the flag would be /MAP:"${CMAKE_CURRENT_BINARY_DIR}/webots.map"
    # For Clang/LLD, it might be -Wl,--cref or -Wl,-Map=webots.map depending on the linker
endif()
# Link main executable with all components
target_link_libraries(webots
    PRIVATE
    app_lib
    gui_lib
    scene_tree_lib
    editor_lib
    engine_lib
    control_lib
    plugins_lib
    nodes_utils_lib
    nodes_lib
    ode_lib
    wren_lib
    sound_lib
    user_commands_lib
    widgets_lib
    vrml_lib
    core_lib
    maths_lib
    util_lib
    Qt6::Core
    Qt6::Gui
    Qt6::Widgets
    Qt6::Network
    Qt6::OpenGL
    Qt6::QmlIntegration
    ${OPENGL_LIBRARIES}
)
