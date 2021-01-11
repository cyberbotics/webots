// Copyright 1996-2021 Cyberbotics Ltd.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*
 * Description:  Defines the entry point of the robot window library
 */

#include "string_utils.h"

#include <stdlib.h>
#include <string.h>

// string_utils_strsep is a string.h strsep version, checking that the delimiter is not preceeded by an escape character ('\').
// To do this, the "pc" variable has been added to https://opensource.apple.com/source/xnu/xnu-1228.7.58/bsd/libkern/strsep.c
char *string_utils_strsep(char **stringp, const char *delim) {
  char *s;
  int sc, pc = 0;
  char *tok;

  if ((s = *stringp) == NULL)
    return (NULL);
  for (tok = s;;) {
    const int c = *s++;
    const char *spanp = delim;
    do {
      if ((sc = *spanp++) == c && pc != '\\') {
        if (c == 0)
          s = NULL;
        else
          s[-1] = 0;
        *stringp = s;
        return (tok);
      }
    } while (sc != 0);
    pc = c;
  }
}

// ref. https://stackoverflow.com/a/779960/2210777
char *string_utils_replace(char *orig, char *rep, char *with) {
  char *result, *ins, *tmp;
  int len_rep, len_with, count;

  if (!orig || !rep)
    return NULL;
  len_rep = strlen(rep);
  if (len_rep == 0)
    return NULL;
  if (!with)
    with = "";
  len_with = strlen(with);

  ins = orig;
  for (count = 0; (tmp = strstr(ins, rep)); ++count)
    ins = tmp + len_rep;

  tmp = result = malloc(strlen(orig) + (len_with - len_rep) * count + 1);
  if (!result)
    return NULL;

  while (count--) {
    ins = strstr(orig, rep);
    const int len_front = ins - orig;
    tmp = strncpy(tmp, orig, len_front) + len_front;
    tmp = strcpy(tmp, with) + len_with;
    orig += len_front + len_rep;
  }
  strcpy(tmp, orig);
  return result;
}
