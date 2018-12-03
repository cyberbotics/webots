#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

#define BUFFER_SIZE 256

int main(int argc, char *argv[]) {
  char c, buffer[BUFFER_SIZE], pattern[BUFFER_SIZE];
  snprintf(pattern, BUFFER_SIZE, "Compressing: %s\\", getenv("WEBOTS_HOME"));
  int i = 0, l = strlen(pattern);
  int column = 80;
  if (argc > 1)
    sscanf(argv[1], "%d", &column);
  for(;;) {
    c = getchar();
    if (c == EOF)
      break;
    switch(c) {
    case 10:
      break;
    case 13: // EOL
      buffer[i] = '\0';
      if (strncmp(buffer, pattern, l) == 0) {
        snprintf(buffer, BUFFER_SIZE, "Adding %s", &buffer[l]);
        i = strlen(buffer);
      }
      if (i > column - 2)
        i = column - 2;
      else while (i < column - 2)
        buffer[i++] = ' ';
      buffer[i++] = 13;
      buffer[i++] = '\0';
      buffer[0] = tolower(buffer[0]);
      printf("# %s",buffer);
      fflush(stdout);
      i = 0;
      break;
    default:
      if (i < BUFFER_SIZE - 2)
        buffer[i++] = c;
      break;
    }
  }
  return 0;
}
