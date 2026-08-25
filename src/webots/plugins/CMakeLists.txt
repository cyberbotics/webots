add_webots_library(plugins_lib)

target_link_libraries(plugins_lib 
    PRIVATE 
    core_lib
)
