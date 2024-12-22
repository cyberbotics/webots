add_subdirectory(utils)
add_webots_library(nodes_lib)

# Enable RTTI and visibility
target_compile_options(nodes_lib
    PUBLIC
    -frtti
    -fvisibility=default
    -fno-gnu-unique
)

# Set position independent code
set_property(TARGET nodes_lib PROPERTY POSITION_INDEPENDENT_CODE ON)

# Ensure symbols are exported
set_target_properties(nodes_lib PROPERTIES
    ENABLE_EXPORTS ON
    WINDOWS_EXPORT_ALL_SYMBOLS ON
    CXX_VISIBILITY_PRESET default
    VISIBILITY_INLINES_HIDDEN OFF
)

target_link_libraries(nodes_lib 
    PUBLIC 
    nodes_utils_lib
    core_lib
    vrml_lib
    ${CMAKE_DL_LIBS}
)
