include(ExternalProject)

# Determine library suffix based on platform
if(WIN32)
    set(PICO_LIB_NAME "libpico.dll")
    set(PICO_IMPLIB_NAME "libpico.dll.a")
elseif(APPLE)
    set(PICO_LIB_NAME "libpico.dylib")
else()
    set(PICO_LIB_NAME "libpico.so")
endif()

# Set paths
set(PICOTTS_PREFIX ${CMAKE_BINARY_DIR}/external/picotts)
set(PICOTTS_LIBRARY ${PICOTTS_PREFIX}/lib/${PICO_LIB_NAME})
set(PICOTTS_INCLUDE_DIR ${PICOTTS_PREFIX}/include)

if(WIN32)
    set(PICOTTS_IMPLIB ${PICOTTS_PREFIX}/lib/${PICO_IMPLIB_NAME})
endif()

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
        -DCMAKE_C_COMPILER=${CMAKE_C_COMPILER}
        -DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER}
    BUILD_BYPRODUCTS 
        ${PICOTTS_LIBRARY}
        ${PICOTTS_IMPLIB}
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
if(WIN32)
    set_target_properties(pico::pico PROPERTIES
        IMPORTED_LOCATION "${PICOTTS_LIBRARY}"
        IMPORTED_IMPLIB "${PICOTTS_IMPLIB}"
        INTERFACE_INCLUDE_DIRECTORIES "${PICOTTS_INCLUDE_DIR}"
    )
else()
    set_target_properties(pico::pico PROPERTIES
        IMPORTED_LOCATION "${PICOTTS_LIBRARY}"
        INTERFACE_INCLUDE_DIRECTORIES "${PICOTTS_INCLUDE_DIR}"
    )
endif()

# Create a custom target that depends on the external project
add_custom_target(picotts ALL DEPENDS picotts_external)
add_dependencies(pico::pico picotts)
