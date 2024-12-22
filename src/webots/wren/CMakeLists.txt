# Enable Qt MOC
set(CMAKE_AUTOMOC ON)

file(GLOB SOURCES "*.cpp")
add_webots_library(wren_lib ${SOURCES})

target_include_directories(wren_lib PUBLIC 
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/include
)

target_link_libraries(wren_lib 
    PUBLIC
    Qt6::Core
    Qt6::Gui
    Qt6::OpenGL
    ${OPENGL_LIBRARIES}
    maths_lib
)
