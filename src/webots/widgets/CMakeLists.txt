add_webots_library(widgets_lib)

target_link_libraries(widgets_lib 
    PRIVATE 
    Qt6::Gui
    Qt6::Widgets
    maths_lib
)
