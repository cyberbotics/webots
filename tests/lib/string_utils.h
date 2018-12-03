#ifndef STRING_UTILS_H
#define STRING_UTILS_H

#include <webots/types.h>  // for bool

bool string_starts_with(const char *pre, const char *str) {
  size_t lenpre = strlen(pre), lenstr = strlen(str);
  return lenstr < lenpre ? false : strncmp(pre, str, lenpre) == 0;
}

#endif
