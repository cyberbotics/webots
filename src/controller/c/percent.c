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

#include "percent.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

char *percent_encode(const char *source) {
  const int length = strlen(source);

  char *buffer = malloc(length * 3 + 1);  // maximum size
  int j = 0;
  for (int i = 0; i < length + 1; i++) {
    unsigned char c = source[i];
    if ((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') || c == '\0' || c == '-' || c == '_' ||
        c == '~' || c == '.')
      buffer[j++] = c;
    else {
      int n;
      if (c >= 0xF0)  // UTF-8 sequence with 4 chars
        n = 4;
      else if (c >= 0xE0)  // UTF-8 sequence with 3 chars
        n = 3;
      else if (c >= 0xC0)  // UTF-8 sequence with 2 chars
        n = 2;
      else
        n = 1;
      for (int k = 0; k < n; k++)
        snprintf(&buffer[j + 3 * k], 4, "%%%02X", 0xFF & source[i + k]);
      i += n - 1;
      j += 3 * n;
    }
  }
  char *new_buffer = realloc(buffer, j);  // adjust to used size
  if (new_buffer == NULL)
    free(buffer);
  return new_buffer;
}
