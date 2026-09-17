# Enable Qt MOC
set(CMAKE_AUTOMOC ON)

file(GLOB SOURCES "*.cpp")
add_webots_library(engine_lib ${SOURCES})

target_include_directories(engine_lib PUBLIC 
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/src/webots/core
    ${CMAKE_SOURCE_DIR}/src/webots/nodes
    ${CMAKE_SOURCE_DIR}/src/webots/vrml
    ${CMAKE_SOURCE_DIR}/include
)

target_link_libraries(engine_lib 
    PUBLIC
    Qt6::Core
    Qt6::Gui
    Qt6::Widgets
    core_lib
    nodes_lib
    ode_lib
    vrml_lib
)
