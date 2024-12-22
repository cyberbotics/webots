add_webots_library(control_lib)

target_link_libraries(control_lib 
    PRIVATE 
    core_lib
)
