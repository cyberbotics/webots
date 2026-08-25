add_webots_library(user_commands_lib)

target_link_libraries(user_commands_lib 
    PRIVATE 
    Qt6::Widgets
    editor_lib
)
