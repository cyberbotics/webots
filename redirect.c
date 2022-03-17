// compile with: gcc -Wall redirect.c -o redirect.exe

#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

static int fds[2];

static void read_pipe() {
  char buffer[100];
  fflush(NULL);  // Need to flush the pipe
#ifdef _WIN32
  if (eof(fds[0])) {
    fprintf(stderr, "nothing to read\n");
    return;
  }
#endif
  int len = read(fds[0], buffer, 100);
#ifndef _WIN32
  if (len == -1) {
    fprintf(stderr, "nothing to read\n");
    return;
  }
#endif
  buffer[len] = 0;  // Buffer now contains "Test\r\n"
  fprintf(stderr, "result (%d): %s\n", len, buffer);
}

int main(int argc, char *argv[]) {
#ifdef _WIN32
  _pipe(fds, 1024, O_TEXT);
#else
  if (pipe(fds) == -1) {
    fprintf(stderr, "Cannot pipe stdout\n");
    exit(EXIT_FAILURE);
  }
  fcntl(fds[0], F_SETFL, O_NONBLOCK);
#endif
  dup2(fds[1], 1);  // 1 is stdout

  printf("Test");
  read_pipe();
  // printf("Nothing");
  read_pipe();
  printf("Something");
  read_pipe();
  return 0;
}
