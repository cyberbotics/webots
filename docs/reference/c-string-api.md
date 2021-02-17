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

The `wbu_string_strsep` function locates, in the string referenced by the `stringp` pointer, the first occurrence of any character in the string `delim` and returns the substring found on the left of the delimiter.
Additionally, the `stringp` pointer is updated so that it stores the location of the character after the delimiter character.
This function is an improvement of the existing `strsep` function in `string.h` where delimiter characters preceded by an escape character are ignored.

The `wbu_string_replace` function returns a copy of the original string `value` where all the occurrences of the `before` substring are replaced with the `after` substring.
