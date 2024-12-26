# Find required packages
find_package(Freetype REQUIRED)

# Add glad library
add_library(glad glad/glad.c)
target_include_directories(glad PUBLIC ${CMAKE_SOURCE_DIR}/include)
target_compile_options(glad PUBLIC -fPIC)

add_subdirectory(wren)
add_subdirectory(webots)
add_subdirectory(ode)
