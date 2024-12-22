# ODE (Open Dynamics Engine) CMake configuration

# Create ODE library target
add_library(ode STATIC)

# Add source files
target_sources(ode PRIVATE
    ode/src/array.cpp
    ode/src/box.cpp
    ode/src/capsule.cpp
    ode/src/collision_kernel.cpp
    ode/src/collision_libccd.cpp
    ode/src/collision_quadtreespace.cpp
    ode/src/collision_sapspace.cpp
    ode/src/collision_space.cpp
    ode/src/collision_transform.cpp
    ode/src/collision_trimesh_box.cpp
    ode/src/collision_trimesh_ccylinder.cpp
    ode/src/collision_trimesh_distance.cpp
    ode/src/collision_trimesh_plane.cpp
    ode/src/collision_trimesh_ray.cpp
    ode/src/collision_trimesh_sphere.cpp
    ode/src/collision_trimesh_trimesh.cpp
    ode/src/collision_util.cpp
    ode/src/convex.cpp
    ode/src/cylinder.cpp
    ode/src/error.cpp
    ode/src/export-dif.cpp
    ode/src/heightfield.cpp
    ode/src/lcp.cpp
    ode/src/mass.cpp
    ode/src/mat.cpp
    ode/src/matrix.cpp
    ode/src/memory.cpp
    ode/src/misc.cpp
    ode/src/objects.cpp
    ode/src/obstack.cpp
    ode/src/ode.cpp
    ode/src/odeinit.cpp
    ode/src/odemath.cpp
    ode/src/odeou.cpp
    ode/src/odetls.cpp
    ode/src/plane.cpp
    ode/src/quickstep.cpp
    ode/src/ray.cpp
    ode/src/rotation.cpp
    ode/src/sphere.cpp
    ode/src/step.cpp
    ode/src/timer.cpp
    ode/src/util.cpp
    # Joint sources
    ode/src/joints/amotor.cpp
    ode/src/joints/ball.cpp
    ode/src/joints/contact.cpp
    ode/src/joints/dball.cpp
    ode/src/joints/dhinge.cpp
    ode/src/joints/fixed.cpp
    ode/src/joints/hinge.cpp
    ode/src/joints/hinge2.cpp
    ode/src/joints/joint.cpp
    ode/src/joints/lmotor.cpp
    ode/src/joints/null.cpp
    ode/src/joints/piston.cpp
    ode/src/joints/plane2d.cpp
    ode/src/joints/pr.cpp
    ode/src/joints/pu.cpp
    ode/src/joints/slider.cpp
    ode/src/joints/transmission.cpp
    ode/src/joints/universal.cpp
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
)

# Define compile definitions and options
target_compile_definitions(ode 
    PUBLIC
        dDOUBLE
    PRIVATE
        CCD_DOUBLE
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
