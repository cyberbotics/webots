# Demo executables
add_executable(default_demo default.cpp)
add_executable(phong_demo phong.cpp)

# Link with wren library and other dependencies
target_link_libraries(default_demo 
    PRIVATE
    wren
    ${OPENGL_LIBRARIES}
    Qt6::Core
    Qt6::Gui
    Qt6::OpenGL
    glut
    glad
)

target_link_libraries(phong_demo 
    PRIVATE 
    wren
    glad
    glut
    ${OPENGL_LIBRARIES}
)

# Include directories
target_include_directories(default_demo 
    PRIVATE
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_SOURCE_DIR}/src/webots/external
    ${OPENGL_INCLUDE_DIR}
)

target_include_directories(phong_demo 
    PRIVATE
    ${CMAKE_SOURCE_DIR}/src
)
