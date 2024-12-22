# Enable Qt MOC
set(CMAKE_AUTOMOC ON)

add_webots_library(app_lib)

# Enable RTTI and visibility
target_compile_options(app_lib
    PUBLIC
    -frtti
    -fvisibility=default
    -fno-gnu-unique
)

# Set position independent code
set_property(TARGET app_lib PROPERTY POSITION_INDEPENDENT_CODE ON)

# Ensure symbols are exported
set_target_properties(app_lib PROPERTIES
    ENABLE_EXPORTS ON
    WINDOWS_EXPORT_ALL_SYMBOLS ON
    CXX_VISIBILITY_PRESET default
    VISIBILITY_INLINES_HIDDEN OFF
)

target_link_libraries(app_lib 
    PUBLIC
    nodes_utils_lib
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
