add_webots_library(sound_lib)

target_link_libraries(sound_lib
    PRIVATE
    core_lib
    OpenAL::OpenAL
    pico::pico
    Qt6::Xml
)

