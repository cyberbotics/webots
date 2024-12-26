add_webots_library(sound_lib)
add_dependencies(sound_lib picotts)
target_link_libraries(sound_lib
    PRIVATE
    core_lib
    OpenAL::OpenAL
    pico::pico
    Qt6::Xml
)
