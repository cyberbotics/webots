# Enable Qt MOC
set(CMAKE_AUTOMOC ON)

file(GLOB SOURCES "*.cpp")
add_webots_library(vrml_lib ${SOURCES})

target_include_directories(vrml_lib PUBLIC 
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/src/webots/core
    ${CMAKE_SOURCE_DIR}/src/webots/maths
    ${CMAKE_SOURCE_DIR}/include
)

target_link_libraries(vrml_lib 
    PUBLIC
    Qt6::Core
    Qt6::Gui
    maths_lib
    core_lib
)
