add_subdirectory(utils)
add_webots_library(nodes_lib)

# Enable RTTI
target_compile_options(nodes_lib
    PUBLIC
    -frtti
)

target_link_libraries(nodes_lib 
    PUBLIC 
    core_lib
    vrml_lib
    nodes_utils_lib
)

# Ensure symbols are exported
set_target_properties(nodes_lib PROPERTIES
    ENABLE_EXPORTS ON
    WINDOWS_EXPORT_ALL_SYMBOLS ON
)
