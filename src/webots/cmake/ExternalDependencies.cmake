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

# Find the installed libpico library
find_library(PICOTTS_LIBRARY
    NAMES pico libpico
    PATHS ${CMAKE_BINARY_DIR}/external/picotts/lib ${CMAKE_BINARY_DIR}/external/picotts/<INSTALL_DIR>/lib
    NO_DEFAULT_PATH
)
message(STATUS "PICOTTS_LIBRARY is ${PICOTTS_LIBRARY}")

# Create an IMPORTED target for libpico
if(PICOTTS_LIBRARY)
    add_library(pico::pico UNKNOWN IMPORTED)  # Use SHARED on Linux/macOS
    set_target_properties(pico::pico PROPERTIES
        IMPORTED_LOCATION ${PICOTTS_LIBRARY}
        INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_BINARY_DIR}/external/picotts/src/picotts/pico/lib;${CMAKE_BINARY_DIR}/external/picotts/src/picotts/pico/compat"
    )

    # Add a dependency to ensure picotts is built before we use its library
    add_dependencies(pico::pico picotts)
else()
    message(FATAL_ERROR "Could not find libpico library after building picotts")
endif()
