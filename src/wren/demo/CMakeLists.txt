# Demo executables
add_executable(default_demo default.cpp)
add_executable(phong_demo phong.cpp)

# Link with wren library
target_link_libraries(default_demo PRIVATE wren)
target_link_libraries(phong_demo PRIVATE wren)

# Include directories
target_include_directories(default_demo PRIVATE ${CMAKE_SOURCE_DIR})
target_include_directories(phong_demo PRIVATE ${CMAKE_SOURCE_DIR})
