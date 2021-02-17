#include <webots/utils/string.h>

#include <stdlib.h>
#include <string.h>

// wbu_string_strsep is a string.h strsep version, checking that the delimiter is not preceeded by an escape character
// ('\'). To do this, the "pc" variable has been added to
// https://opensource.apple.com/source/xnu/xnu-1228.7.58/bsd/libkern/strsep.c
char *wbu_string_strsep(char **stringp, const char *delim) {
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
char *wbu_string_replace(char *value, char *before, char *after) {
  char *result, *ins, *tmp;
  int len_before, len_after, count;

  if (!value || !before)
    return NULL;
  len_before = strlen(before);
  if (len_before == 0)
    return NULL;
  if (!after)
    after = "";
  len_after = strlen(after);

  ins = value;
  for (count = 0; (tmp = strstr(ins, before)); ++count)
    ins = tmp + len_before;

  tmp = result = malloc(strlen(value) + (len_after - len_before) * count + 1);
  if (!result)
    return NULL;

  while (count--) {
    ins = strstr(value, before);
    const int len_front = ins - value;
    tmp = strncpy(tmp, value, len_front) + len_front;
    tmp = strcpy(tmp, after) + len_after;
    value += len_front + len_before;
  }
  strcpy(tmp, value);
  return result;
}
