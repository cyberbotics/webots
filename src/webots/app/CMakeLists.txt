add_webots_library(app_lib)

target_link_libraries(app_lib 
    PRIVATE 
    Qt6::Gui
    Qt6::Widgets
    Qt6::Network
    Qt6::OpenGL
)
