add_webots_library(wren_lib)

target_link_libraries(wren_lib 
    PRIVATE 
    Qt6::Gui
    Qt6::OpenGL
    ${OPENGL_LIBRARIES}
    maths_lib
)
