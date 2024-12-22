# ODE (Open Dynamics Engine) CMake configuration

# Create ODE library target
add_library(ode STATIC)

# Find all cpp files recursively
file(GLOB_RECURSE ODE_SOURCES 
    "${CMAKE_CURRENT_SOURCE_DIR}/ode/src/*.cpp"
)

# Add source files
target_sources(ode PRIVATE
    ${ODE_SOURCES}
)

# Add OPCODE collision detection library sources
target_sources(ode PRIVATE
    OPCODE/OPC_AABBCollider.cpp
    OPCODE/OPC_AABBTree.cpp
    OPCODE/OPC_BaseModel.cpp
    OPCODE/OPC_Collider.cpp
    OPCODE/OPC_Common.cpp
    OPCODE/OPC_HybridModel.cpp
    OPCODE/OPC_LSSCollider.cpp
    OPCODE/OPC_MeshInterface.cpp
    OPCODE/OPC_Model.cpp
    OPCODE/OPC_OBBCollider.cpp
    OPCODE/OPC_OptimizedTree.cpp
    OPCODE/OPC_Picking.cpp
    OPCODE/OPC_PlanesCollider.cpp
    OPCODE/OPC_RayCollider.cpp
    OPCODE/OPC_SphereCollider.cpp
    OPCODE/OPC_TreeBuilders.cpp
    OPCODE/OPC_TreeCollider.cpp
    OPCODE/OPC_VolumeCollider.cpp
    OPCODE/Opcode.cpp
    OPCODE/Ice/IceAABB.cpp
    OPCODE/Ice/IceContainer.cpp
    OPCODE/Ice/IceHPoint.cpp
    OPCODE/Ice/IceIndexedTriangle.cpp
    OPCODE/Ice/IceMatrix3x3.cpp
    OPCODE/Ice/IceMatrix4x4.cpp
    OPCODE/Ice/IceOBB.cpp
    OPCODE/Ice/IcePlane.cpp
    OPCODE/Ice/IcePoint.cpp
    OPCODE/Ice/IceRandom.cpp
    OPCODE/Ice/IceRay.cpp
    OPCODE/Ice/IceRevisitedRadix.cpp
    OPCODE/Ice/IceSegment.cpp
    OPCODE/Ice/IceTriangle.cpp
    OPCODE/Ice/IceUtils.cpp
)

# Add libccd collision detection library sources
target_sources(ode PRIVATE
    libccd/src/alloc.c
    libccd/src/ccd.c
    libccd/src/mpr.c
    libccd/src/polytope.c
    libccd/src/support.c
    libccd/src/vec3.c
)

# Set include directories
target_include_directories(ode
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
        $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include/ode>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
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
)

# Set properties
set_target_properties(ode PROPERTIES
    VERSION 0.16.2
    SOVERSION 8
    PUBLIC_HEADER "${CMAKE_CURRENT_SOURCE_DIR}/include/ode/ode.h"
)

# Install rules
include(GNUInstallDirs)
install(TARGETS ode
    EXPORT odeTargets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/ode
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

# Export targets
install(EXPORT odeTargets
    FILE odeTargets.cmake
    NAMESPACE ode::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/ode
)

# Create and install config file
include(CMakePackageConfigHelpers)
configure_package_config_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/odeConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/odeConfig.cmake
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/ode
)

install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/odeConfig.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/ode
)
