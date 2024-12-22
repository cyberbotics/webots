add_subdirectory(utils)
add_webots_library(nodes_lib)

target_link_libraries(nodes_lib 
    PUBLIC 
    core_lib
    vrml_lib
    nodes_utils_lib
)
