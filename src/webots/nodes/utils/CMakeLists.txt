# Enable Qt MOC
set(CMAKE_AUTOMOC ON)

file(GLOB SOURCES "*.cpp")
list(REMOVE_ITEM SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/WbVirtualRealityHeadset.cpp")

if(WIN32)
  list(APPEND SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/WbVirtualRealityHeadset.cpp")
endif()

add_library(nodes_utils_lib ${SOURCES})

target_include_directories(nodes_utils_lib PUBLIC 
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/src/webots/nodes
    ${CMAKE_SOURCE_DIR}/src/webots/core
    ${CMAKE_SOURCE_DIR}/src/webots/vrml
    ${CMAKE_SOURCE_DIR}/src/webots/util
    ${CMAKE_SOURCE_DIR}/include
    $ENV{MINGW_PREFIX}/include
    $ENV{MINGW_PREFIX}/local/include
    /usr/include
    /usr/local/include
)

target_link_libraries(nodes_utils_lib
    PUBLIC
    Qt6::Core
    Qt6::Gui
    Qt6::Widgets
    vrml_lib
    util_lib
    core_lib
    ${OIS_LIBRARIES}
    PkgConfig::OIS
    PkgConfig::ODE
)
