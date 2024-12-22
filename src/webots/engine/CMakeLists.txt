add_webots_library(engine_lib)

target_link_libraries(engine_lib 
    PRIVATE 
    core_lib
    nodes_lib
    ode_lib
)
