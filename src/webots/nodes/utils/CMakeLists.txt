# Enable Qt MOC
set(CMAKE_AUTOMOC ON)

file(GLOB SOURCES "*.cpp")
list(REMOVE_ITEM SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/WbVirtualRealityHeadset.cpp")

if(WIN32)
  list(APPEND SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/WbVirtualRealityHeadset.cpp")
endif()

add_library(nodes_utils_lib SHARED ${SOURCES})

# Enable RTTI and visibility
target_compile_options(nodes_utils_lib
    PUBLIC
    -frtti
    -fvisibility=default
    -fno-gnu-unique
)

# Set position independent code
set_property(TARGET nodes_utils_lib PROPERTY POSITION_INDEPENDENT_CODE ON)

# Ensure symbols are exported
set_target_properties(nodes_utils_lib PROPERTIES
    ENABLE_EXPORTS ON
    WINDOWS_EXPORT_ALL_SYMBOLS ON
    CXX_VISIBILITY_PRESET default
    VISIBILITY_INLINES_HIDDEN OFF
    BUILD_SHARED_LIBS ON
)

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
    PRIVATE vrml_lib
    PRIVATE util_lib  
    PRIVATE core_lib
    PUBLIC Qt6::Core
    PUBLIC Qt6::Gui
    PUBLIC Qt6::Widgets
    PUBLIC ${OIS_LIBRARIES}
    PUBLIC PkgConfig::OIS
    PUBLIC PkgConfig::ODE
)

# Ensure all objects in this library and its dependencies are built with -fPIC
set_property(TARGET nodes_utils_lib PROPERTY POSITION_INDEPENDENT_CODE ON)
set_property(TARGET vrml_lib PROPERTY POSITION_INDEPENDENT_CODE ON) 
set_property(TARGET util_lib PROPERTY POSITION_INDEPENDENT_CODE ON)
set_property(TARGET core_lib PROPERTY POSITION_INDEPENDENT_CODE ON)
