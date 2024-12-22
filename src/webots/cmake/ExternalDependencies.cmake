include(ExternalProject)

# PicoTTS
ExternalProject_Add(
    picotts
    GIT_REPOSITORY https://github.com/cyberbotics/picotts.git
    GIT_TAG master
    PREFIX ${CMAKE_BINARY_DIR}/external/picotts
    CONFIGURE_COMMAND cd <SOURCE_DIR>/pico && ./autogen.sh && ./configure --prefix=<INSTALL_DIR>
    BUILD_COMMAND cd <SOURCE_DIR>/pico && make
    BUILD_IN_SOURCE 0
    INSTALL_COMMAND ""  # Skip install step
)

# Set variables for include and lib paths
set(PICOTTS_INCLUDE_DIR ${CMAKE_BINARY_DIR}/external/picotts/src/picotts/pico/lib)
set(PICOTTS_LIB_DIR ${CMAKE_BINARY_DIR}/external/picotts/src/picotts/pico/lib/.libs)
