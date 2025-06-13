add_webots_library(editor_lib)


target_link_libraries(editor_lib 
    PRIVATE 
    Qt6::Gui
    Qt6::Widgets
    Qt6::PrintSupport Qt6::Widgets
    core_lib
)
