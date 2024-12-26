# Wren library configuration

# Source files
file(GLOB WREN_SOURCES 
    "*.cpp"
)

# Header files
file(GLOB WREN_HEADERS
    "*.hpp"
)

# Create wren library
add_library(wren STATIC ${WREN_SOURCES} ${WREN_HEADERS})
target_compile_options(wren PUBLIC -fPIC)

# Find required packages
find_package(Freetype REQUIRED)
find_package(assimp REQUIRED)

# Include directories
target_include_directories(wren PUBLIC 
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/src/webots/external
    ${CMAKE_SOURCE_DIR}/src/webots/external/siphash
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/src/webots/core
    ${OPENGL_INCLUDE_DIR}
    ${Qt6Core_INCLUDE_DIRS}
    ${Qt6Gui_INCLUDE_DIRS}
    ${Qt6OpenGL_INCLUDE_DIRS}
    ${FREETYPE_INCLUDE_DIRS}
    ${ASSIMP_INCLUDE_DIRS}
)

# Define GLM experimental features
add_definitions(-DGLM_ENABLE_EXPERIMENTAL)

# Link libraries
target_link_libraries(wren PUBLIC
    ${OPENGL_LIBRARIES}
    Qt6::Core
    Qt6::Gui
    Qt6::OpenGL
    ${FREETYPE_LIBRARIES}
    assimp
    glad
)

# Demo subdirectory
add_subdirectory(demo)
