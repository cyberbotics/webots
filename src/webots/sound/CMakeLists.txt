add_webots_library(sound_lib)
add_dependencies(sound_lib picotts)

# Find SAPI (Speech API)
if (WIN32)
    # Directly use MSYSTEM_PREFIX
    set(SAPI_LIBRARY_PATH "$ENV{MSYSTEM_PREFIX}/lib/libsapi.a")

    # Check if SAPI library exists and link it
    if (EXISTS ${SAPI_LIBRARY_PATH})
        target_link_libraries(sound_lib
            PRIVATE
            core_lib
            OpenAL::OpenAL
            pico::pico
            Qt6::Xml
            ${SAPI_LIBRARY_PATH}
        )
        message(STATUS "SAPI library found and linked: ${SAPI_LIBRARY_PATH}")
    else()
        message(FATAL_ERROR "SAPI library not found at ${SAPI_LIBRARY_PATH}. Please ensure it is installed in your MSYS2 environment.")
    endif()
else()
    target_link_libraries(sound_lib
        PRIVATE
        core_lib
        OpenAL::OpenAL
        pico::pico
        Qt6::Xml
    )
endif()

