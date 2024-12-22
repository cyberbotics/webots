#ifndef PICO_TTS_WRAPPER_H
#define PICO_TTS_WRAPPER_H

// Rename C keyword 'this' to avoid conflicts
#define this pico_this

#ifdef __cplusplus
extern "C" {
#endif

#include <picoapi.h>
#include <picoapid.h>
#include <picoos.h>

#ifdef __cplusplus
}
#endif

#undef this

#endif // PICO_TTS_WRAPPER_H
