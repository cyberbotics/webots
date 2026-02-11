add_webots_library(scene_tree_lib)

target_link_libraries(scene_tree_lib 
    PRIVATE 
    Qt6::Widgets
    nodes_lib
)
