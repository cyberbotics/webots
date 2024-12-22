# Enable Qt MOC
set(CMAKE_AUTOMOC ON)

add_webots_library(app_lib)

# Enable RTTI to match nodes_lib
target_compile_options(app_lib
    PUBLIC
    -frtti
)

# Set position independent code
set_property(TARGET app_lib PROPERTY POSITION_INDEPENDENT_CODE ON)

target_link_libraries(app_lib 
    PUBLIC
    nodes_lib
    Qt6::Core
    Qt6::Gui
    Qt6::Widgets
    Qt6::Network
    Qt6::OpenGL
    Qt6::QmlIntegration
    engine_lib
    wren_lib
    ode
)
