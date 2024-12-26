add_webots_library(core_lib)
target_link_libraries(core_lib
    PRIVATE
    Qt6::Qml
    Qt6::Test
)

