add_subdirectory(utils)
add_webots_library(nodes_lib)

# Enable RTTI and visibility
target_compile_options(nodes_lib
    PUBLIC
    -frtti
    -fvisibility=default
)
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    add_compile_options(-fno-gnu-unique)
endif()

# Find STB headers
find_path(STB_INCLUDE_DIR
    NAMES stb_image.h
    PATHS
        ${STB_ROOT}/include
        ${STB_ROOT}
        $ENV{MSYSTEM_PREFIX}/include/stb
        $ENV{MSYSTEM_PREFIX}/local/include/stb
        /usr/include/stb
        /usr/local/include/stb
        ${CMAKE_SOURCE_DIR}/external/stb
        ${CMAKE_SOURCE_DIR}/src/webots/external/stb
        ${CMAKE_SOURCE_DIR}/include/stb
)

if(NOT STB_INCLUDE_DIR)
    message(FATAL_ERROR "STB headers not found - please install stb package or set STB_ROOT")
else()
    message(STATUS "Found STB headers at: ${STB_INCLUDE_DIR}")
endif()

# Set position independent code
set_property(TARGET nodes_lib PROPERTY POSITION_INDEPENDENT_CODE ON)

# Ensure symbols are exported
set_target_properties(nodes_lib PROPERTIES
    ENABLE_EXPORTS ON
    WINDOWS_EXPORT_ALL_SYMBOLS ON
    CXX_VISIBILITY_PRESET default
    VISIBILITY_INLINES_HIDDEN OFF
)

target_include_directories(nodes_lib
    PUBLIC
    ${STB_INCLUDE_DIR}
)

target_link_libraries(nodes_lib
    PUBLIC 
    core_lib
    vrml_lib
    ${CMAKE_DL_LIBS}
)
