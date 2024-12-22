# Helper function to get source files
function(get_source_files DIR SOURCES_OUT)
    file(GLOB SOURCES "${DIR}/*.cpp")
    set(${SOURCES_OUT} ${SOURCES} PARENT_SCOPE)
endfunction()

# Helper function to create a library with standard settings
function(add_webots_library LIB_NAME)
    get_filename_component(DIR_NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)
    
    # Get all cpp files
    file(GLOB ALL_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp")
    
    # Filter out Windows-specific files on non-Windows platforms
    if(NOT WIN32)
        list(FILTER ALL_SOURCES EXCLUDE REGEX "WbVirtualRealityHeadset\\.cpp$")
        list(FILTER ALL_SOURCES EXCLUDE REGEX "WbMicrosoftTextToSpeech\\.cpp$")
        list(FILTER ALL_SOURCES EXCLUDE REGEX "WbWindowsRegistry\\.cpp$")
    endif()
    
    add_library(${DIR_NAME}_lib ${ALL_SOURCES})
    target_include_directories(${DIR_NAME}_lib PUBLIC 
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_SOURCE_DIR}/include
        ${CMAKE_SOURCE_DIR}/src/webots/core
        ${CMAKE_SOURCE_DIR}/src/webots/vrml
        ${CMAKE_SOURCE_DIR}/src/webots/util
        ${CMAKE_SOURCE_DIR}/src/webots/widgets
        ${CMAKE_SOURCE_DIR}/src/webots/external/compilation_timestamp
        ${CMAKE_SOURCE_DIR}/src/webots/user_commands
        ${CMAKE_SOURCE_DIR}/src/webots/maths
        ${CMAKE_SOURCE_DIR}/src/webots/nodes/utils
        ${CMAKE_SOURCE_DIR}/src/webots/wren
        ${CMAKE_SOURCE_DIR}/src/webots/ode
        ${CMAKE_SOURCE_DIR}/src/webots/nodes
        ${CMAKE_SOURCE_DIR}/src/webots/app
        ${CMAKE_SOURCE_DIR}/src/webots/plugins
        ${CMAKE_SOURCE_DIR}/src/webots/engine
        ${CMAKE_SOURCE_DIR}/src/webots/control
        ${CMAKE_SOURCE_DIR}/src/webots/editor
        ${CMAKE_SOURCE_DIR}/src/webots/scene_tree
        ${CMAKE_SOURCE_DIR}/include
        ${CMAKE_SOURCE_DIR}/include/ode
        ${CMAKE_SOURCE_DIR}/src/webots/sound
        ${CMAKE_SOURCE_DIR}/src/webots/external/siphash
        ${CMAKE_SOURCE_DIR}/include/controller/c
        ${PICOTTS_INCLUDE_DIR}
        /usr/include/stb
        /usr/local/include/stb
        $ENV{MINGW_PREFIX}/include/stb
        ${FREETYPE_INCLUDE_DIRS}
    )
    target_link_libraries(${DIR_NAME}_lib PRIVATE Qt6::Core)
    
    # Set C files to be compiled as C
    set_source_files_properties(
        ${CMAKE_BINARY_DIR}/external/picotts/src/picotts/pico/lib/*.c
        PROPERTIES
        COMPILE_FLAGS "-x c"
    )
endfunction()
