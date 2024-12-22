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

# Find Freetype package
find_package(Freetype REQUIRED)

# Include directories
target_include_directories(wren PUBLIC 
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/src/webots/external
    ${CMAKE_SOURCE_DIR}/src/webots/external/siphash
    ${OPENGL_INCLUDE_DIR}
    ${Qt6Core_INCLUDE_DIRS}
    ${Qt6Gui_INCLUDE_DIRS}
    ${Qt6OpenGL_INCLUDE_DIRS}
    ${FREETYPE_INCLUDE_DIRS}
)

# Link libraries
target_link_libraries(wren PUBLIC
    ${OPENGL_LIBRARIES}
    Qt6::Core
    Qt6::Gui
    Qt6::OpenGL
    ${FREETYPE_LIBRARIES}
)

# Demo subdirectory
add_subdirectory(demo)
