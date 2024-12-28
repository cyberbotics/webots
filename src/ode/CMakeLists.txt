# ODE (Open Dynamics Engine) CMake configuration

# ODE (Open Dynamics Engine) CMake configuration

# Create ODE library target
add_library(ode STATIC)

# Find all cpp files recursively in ode/src and its subdirectories
file(GLOB_RECURSE ODE_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/ode/src/*.cpp"
)

# Find all c files recursively in ode/src and its subdirectories
file(GLOB_RECURSE ODE_C_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/ode/src/*.c"
)

# Find all OPCODE collision detection library sources
file(GLOB_RECURSE OPCODE_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/OPCODE/*.cpp"
)


# Find all libccd collision detection library sources
target_sources(ode PRIVATE
    libccd/src/alloc.c
    libccd/src/ccd.c
    libccd/src/mpr.c
    libccd/src/polytope.c
    libccd/src/support.c
    libccd/src/vec3.c
)

# Add source files
target_sources(ode PRIVATE
    ${ODE_SOURCES}
    ${ODE_C_SOURCES}
    ${OPCODE_SOURCES}
)

# Set include directories
target_include_directories(ode
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
        $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include/ode>
        $<INSTALL_INTERFACE:include>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/ode/src
        ${CMAKE_CURRENT_SOURCE_DIR}/OPCODE
        ${CMAKE_CURRENT_SOURCE_DIR}/OPCODE/Ice
        ${CMAKE_CURRENT_SOURCE_DIR}/libccd/src
        ${CMAKE_CURRENT_SOURCE_DIR}/libccd/src/custom
)

# Define compile definitions and options
target_compile_definitions(ode
    PRIVATE
        dTRIMESH_ENABLED
        dTRIMESH_OPCODE
)

target_compile_options(ode PRIVATE
    -I${CMAKE_SOURCE_DIR}/include
    -fPIC
)

# Set properties
set_target_properties(ode PROPERTIES
    VERSION 0.16.2
    SOVERSION 8
    PUBLIC_HEADER "${CMAKE_SOURCE_DIR}/include/ode/ode.h"
)

# Install rules
include(GNUInstallDirs)
install(TARGETS ode
    EXPORT odeTargets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    #PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/ode
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

# Export targets
#install(EXPORT odeTargets
#    FILE odeTargets.cmake
#    NAMESPACE ode::
#    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/ode
#)
#
## Create and install config file
#include(CMakePackageConfigHelpers)
#configure_package_config_file(
#    ${CMAKE_CURRENT_SOURCE_DIR}/odeConfig.cmake.in
#    ${CMAKE_CURRENT_BINARY_DIR}/odeConfig.cmake
#    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/ode
#)
#
#install(FILES
#    ${CMAKE_CURRENT_BINARY_DIR}/odeConfig.cmake
#    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/ode
#)
