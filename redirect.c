// compile with: gcc -Wall redirect.c -o redirect.exe
#include <fcntl.h>
#include <stdio.h>

static int fds[2];

static void read_pipe() {
  char buffer[100];
  fflush(NULL);  // Need to flush the pipe
  if (eof(fds[0])) {
    fprintf(stderr, "nothing to read\n");
    return;
  }
  int len = read(fds[0], buffer, 100);
  buffer[len] = 0;  // Buffer now contains "Test\r\n"
  fprintf(stderr, "result (%d): %s\n", len, buffer);
}

int main(int argc, char *argv[]) {
  _pipe(fds, 1024, O_TEXT);
  dup2(fds[1], 1);  // 1 is stdout

  printf("Test");
  read_pipe();
  // printf("Nothing");
  read_pipe();
  printf("Something");
  read_pipe();
  return 0;
}
