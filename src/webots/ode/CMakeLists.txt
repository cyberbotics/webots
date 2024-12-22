add_webots_library(ode_lib)

target_link_libraries(ode_lib 
    PRIVATE 
    maths_lib
)
