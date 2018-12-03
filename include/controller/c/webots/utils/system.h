/*
 * Copyright 1996-2018 Cyberbotics Ltd.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/************************************************************************************/
/* Description:  Webots C programming interface for cross-platform system functions */
/************************************************************************************/

#ifndef WBU_SYSTEM_H
#define WBU_SYSTEM_H

#ifdef __cplusplus
extern "C" {
#endif

// wbu_system_getenv() returns an UTF-8 multi-byte string for the specified
// environment variable.
// The return value points to a char buffer which may be overwritten by any subsequent
// call to a wbu_system function.
const char *wbu_system_getenv(const char *variable);

// On Linux and macOS, wbu_system_short_path() returns the variable passed as an
// argument. On Windows, it returns an ASCII string corresponding to the Windows 8.3
// short path (see GetShortPathName in the Windows API). This is useful when using a
// library that doesn't support UTF-8 multi-byte strings or wide characters for paths.
// The return value points to a char buffer which may be overwritten by any subsequent
// call to a wbu_system function.
const char *wbu_system_short_path(const char *path);

#ifdef __cplusplus
}
#endif

#endif /* WBU_SYSTEM_H */
