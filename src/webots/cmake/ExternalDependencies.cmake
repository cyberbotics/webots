include(ExternalProject)

# PicoTTS
ExternalProject_Add(
    picotts
    GIT_REPOSITORY https://github.com/Kreijstal/picotts.git
    GIT_TAG master
    PREFIX ${CMAKE_BINARY_DIR}/external/picotts
    SOURCE_SUBDIR pico
    CMAKE_ARGS 
        -DCMAKE_INSTALL_PREFIX=<INSTALL_DIR>
    BUILD_IN_SOURCE 0
)

# Set variables for include and lib paths
set(PICOTTS_INCLUDE_DIR 
    ${CMAKE_BINARY_DIR}/external/picotts/src/picotts/pico/lib
    ${CMAKE_BINARY_DIR}/external/picotts/src/picotts/pico/compat
)
set(PICOTTS_LIB_DIR ${CMAKE_BINARY_DIR}/external/picotts/lib)

# Add compile definitions and flags for PicoTTS
add_compile_definitions(PICO_EXTERN_C)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fno-strict-aliasing -D__STDC_VERSION__=199901L")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fno-strict-aliasing")
