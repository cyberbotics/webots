add_webots_library(util_lib)

target_link_libraries(util_lib 
    PRIVATE 
    Qt6::Network
    Qt6::Widgets
)
