## C String API

### String Utility Functions

The following string utility functions to parse and generate messages are defined in the Webots C API.

#### `wbu_string_strsep`
#### `wbu_string_replace`

```c
#include <webots/utils/string.h>

char *wbu_string_strsep(char **stringp, const char *delim);
char *wbu_string_replace(char *value, char *before, char *after);
```

##### Description

*parse and generate string messages*

The `wbu_string_strsep` function locates, in the string referenced by the `stringp` pointer, the first occurrence of any character in the string `delim`.
If `stringp` is NULL, the strsep() function returns NULL and does nothing else.
Otherwise, this function finds the first token in the string `stringp`, that is delimited by one of the bytes in the string `delim`.
This token is terminated by overwriting the delimiter with a null byte ('\0'), and `stringp` is updated to point past the token.
In case no delimiter was found, the token is taken to be the entire string `stringp`, and `stringp` is made NULL.
This function is an improvement of the standard [strsep](https://man7.org/linux/man-pages/man3/strsep.3.html) function in `string.h` where delimiter characters preceded by an escape character (backslash) are ignored.

The `wbu_string_replace` function returns a copy of the original string `value` where all the occurrences of the `before` substring are replaced with the `after` substring.
The returned string is allocated with `malloc` within the `wbu_string_replace` function and should be freed with `free` by the user when not needed anymore.
