add_webots_library(core_lib)

# Find libraries on Windows (using MSYS2)
if (WIN32)
    # Directly use MSYSTEM_PREFIX for library paths
    set(D3D9_LIBRARY_PATH "$ENV{MSYSTEM_PREFIX}/lib/libd3d9.a")
    set(IPHLPAPI_LIBRARY_PATH "$ENV{MSYSTEM_PREFIX}/lib/libiphlpapi.a")

    # Check if D3D9 library exists and link it
    if (EXISTS ${D3D9_LIBRARY_PATH})
        message(STATUS "D3D9 library found: ${D3D9_LIBRARY_PATH}")
    else()
        message(FATAL_ERROR "D3D9 library not found at ${D3D9_LIBRARY_PATH}. Please ensure it is installed in your MSYS2 environment.")
    endif()

    # Check if IPHLPAPI library exists and link it
    if (EXISTS ${IPHLPAPI_LIBRARY_PATH})
        message(STATUS "IPHLPAPI library found: ${IPHLPAPI_LIBRARY_PATH}")
    else()
        message(FATAL_ERROR "IPHLPAPI library not found at ${IPHLPAPI_LIBRARY_PATH}. Please ensure it is installed in your MSYS2 environment.")
    endif()

    # Link libraries if found
    target_link_libraries(core_lib
        PRIVATE
        Qt6::Qml
        Qt6::Test
        ${D3D9_LIBRARY_PATH}
        ${IPHLPAPI_LIBRARY_PATH}
    )
else()
    target_link_libraries(core_lib
        PRIVATE
        Qt6::Qml
        Qt6::Test
    )
endif()
