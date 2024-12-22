# Enable Qt MOC
set(CMAKE_AUTOMOC ON)

add_webots_library(core_lib)

target_link_libraries(core_lib 
    PRIVATE 
    Qt6::Core
    Qt6::Network
    Qt6::QmlIntegration
)
