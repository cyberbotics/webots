/*
 * Copyright 1996-2023 Cyberbotics Ltd.
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
/* Description:  Webots C utility to parse messages                               */
/**********************************************************************************/

#ifndef WBU_STRING_H
#define WBU_STRING_H

#ifdef __cplusplus
extern "C" {
#endif

char *wbu_string_strsep(char **stringp, const char *delim);
char *wbu_string_replace(char *value, const char *before, const char *after);

#ifdef __cplusplus
}
#endif

#endif  // WBU_STRING_H
