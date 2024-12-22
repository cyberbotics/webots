add_webots_library(nodes_lib)

target_link_libraries(nodes_lib 
    PRIVATE 
    core_lib
    vrml_lib
)
