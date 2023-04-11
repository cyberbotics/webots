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

#ifndef PRINTF_OVERRIDE_H
#define PRINTF_OVERRIDE_H

#include <string.h>
#include <stdio.h>

/*
 * - stdout is redirected on the bluetooth device
 * - if the bluetooth emits something strange continously, uploading
 *   a new crosscompiled firmware is quite impossible.
 * For these reasons, the printf function is silently overriden and do nothing
 */
#define printf printf_override
int printf_override(const char *fmt,...);

#endif /* PRINTF_OVERRIDE_H */
