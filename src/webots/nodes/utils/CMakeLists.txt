# Enable Qt MOC
set(CMAKE_AUTOMOC ON)

add_webots_library(nodes_utils_lib)

target_link_libraries(nodes_utils_lib
    PUBLIC
    Qt6::Core
    Qt6::Gui
    Qt6::Widgets
)
