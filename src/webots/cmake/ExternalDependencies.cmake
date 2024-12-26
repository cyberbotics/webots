include(ExternalProject)

# Determine library suffix based on platform
if(WIN32)
    set(PICO_LIB_NAME "libpico.dll")
elseif(APPLE)
    set(PICO_LIB_NAME "libpico.dylib")
else()
    set(PICO_LIB_NAME "libpico.so")
endif()

# Set paths
set(PICOTTS_PREFIX ${CMAKE_BINARY_DIR}/external/picotts)
set(PICOTTS_LIBRARY ${PICOTTS_PREFIX}/lib/${PICO_LIB_NAME})
set(PICOTTS_INCLUDE_DIR ${PICOTTS_PREFIX}/include)

# Create necessary directories
file(MAKE_DIRECTORY ${PICOTTS_INCLUDE_DIR})

# PicoTTS
ExternalProject_Add(
    picotts_external
    GIT_REPOSITORY https://github.com/Kreijstal/picotts.git
    GIT_TAG master
    PREFIX ${PICOTTS_PREFIX}
    SOURCE_SUBDIR pico
    CMAKE_ARGS
        -DCMAKE_INSTALL_PREFIX=${PICOTTS_PREFIX}
        -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
    BUILD_BYPRODUCTS ${PICOTTS_LIBRARY}
    # Copy header files after build
    INSTALL_COMMAND 
        ${CMAKE_COMMAND} -E copy_directory <SOURCE_DIR>/pico/lib ${PICOTTS_INCLUDE_DIR}
    COMMAND
        ${CMAKE_COMMAND} -E copy_directory <SOURCE_DIR>/pico/compat ${PICOTTS_INCLUDE_DIR}
    COMMAND
        ${CMAKE_MAKE_PROGRAM} install
)

# Create the imported library target
add_library(pico::pico SHARED IMPORTED GLOBAL)

# Set properties on the target
set_target_properties(pico::pico PROPERTIES
    IMPORTED_LOCATION ${PICOTTS_LIBRARY}
    INTERFACE_INCLUDE_DIRECTORIES ${PICOTTS_INCLUDE_DIR}
)

# Create a custom target that depends on the external project
add_custom_target(picotts ALL DEPENDS picotts_external)
add_dependencies(pico::pico picotts)
