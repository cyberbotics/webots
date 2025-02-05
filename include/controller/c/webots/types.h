/*
 * Copyright 1996-2024 Cyberbotics Ltd.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     https://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**********************************************************************************/
/* Description:  Common definitions for both the C and C++ APIs                   */
/**********************************************************************************/

#if defined(WB_USING_CPP_API) && defined(WB_USING_C_API) && !defined(WB_ALLOW_MIXING_C_AND_CPP_API)
#ifdef _MSC_VER
#pragma message("warning: mixing the C and C++ APIs in the same controller is not supported.")
#else
#warning "mixing the C and C++ APIs in the same controller is not supported."
#endif
#endif

#ifndef WB_TYPES_H
#define WB_TYPES_H

// There can be a maximum of 65534 devices on a robot (65535 being reserved)
typedef unsigned short WbDeviceTag;  // identifier of a device

// Opaque type definitions
typedef struct WbImageStructPrivate *WbImageRef;
typedef struct WbMotionStructPrivate *WbMotionRef;
typedef struct WbNodeStructPrivate *WbNodeRef;
typedef struct WbProtoInfoStructPrivate *WbProtoRef;
typedef struct WbFieldStructPrivate *WbFieldRef;

// define "bool" type for C controllers
// C++ code will use the standard definition of "bool"
#ifndef __cplusplus

#include <math.h>  // definition of INFINITY
#ifndef INFINITY
#define INFINITY (1.0 / 0.0)
#endif

#ifndef bool
#define bool char
#endif

#ifndef true
// clang-format off
#define true ((bool)1)
// clang-format on
#endif

#ifndef false
// clang-format off
#define false ((bool)0)
// clang-format on
#endif

#endif

#define WB_ANGULAR 0  // kept for backward compatibility R2018b
typedef enum { WB_ROTATIONAL = 0, WB_LINEAR } WbJointType;

// Allow us to mark functions as 'deprecated' and have gcc emit a nice warning for each use.
// Usage: int foo(char) WB_DEPRECATED;
// and then gcc will emit a warning for each usage of the function.
#ifndef WB_DEPRECATED
#if __GNUC__ >= 3 && !defined WB_MATLAB_LOADLIBRARY
#define WB_DEPRECATED __attribute__((deprecated))
#else
#define WB_DEPRECATED
#endif
#endif

#endif /* WB_TYPES_H */
