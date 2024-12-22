# Enable Qt MOC
set(CMAKE_AUTOMOC ON)

add_webots_library(app_lib)

target_link_libraries(app_lib 
    PUBLIC
    Qt6::Core
    Qt6::Gui
    Qt6::Widgets
    Qt6::Network
    Qt6::OpenGL
    Qt6::QmlIntegration
    engine_lib
    wren_lib
    ode
    nodes_lib
)
