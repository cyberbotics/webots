#ifndef FILE_UTILS_H
#define FILE_UTILS_H

#include <webots/types.h>  // for bool

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

bool file_exists(const char *filename) {
  return access(filename, F_OK) != -1;
}

bool move_file(const char *source_filename, const char *destination_filename) {
  return !rename(source_filename, destination_filename);
}

bool remove_file(const char *filename) {
  return !remove(filename);
}

char *get_file_content(const char *filename) {
  char *buffer = NULL;
  long length;

  FILE *f = fopen(filename, "rb");
  if (f) {
    fseek(f, 0, SEEK_END);
    length = ftell(f);
    fseek(f, 0, SEEK_SET);
    buffer = malloc(length + 1);
    if (buffer) {
      size_t n = fread(buffer, 1, length, f);
      if (n != length) {
        free(buffer);
        return NULL;
      }
      buffer[length] = 0;
    }
    fclose(f);
  }
  return buffer;
}

bool compare_file_content(const char *fn1, const char *fn2) {
  FILE *f1 = fopen(fn1, "r");
  FILE *f2 = fopen(fn1, "r");

  if (f1 == NULL || f2 == NULL)
    return false;

  char c1, c2;
  do {
    c1 = fgetc(f1);
    c2 = fgetc(f2);

    if (c1 != c2) {
      fclose(f1);
      fclose(f2);
      return false;
    }
  } while (c1 != EOF && c2 != EOF);

  fclose(f1);
  fclose(f2);

  return c1 == EOF && c2 == EOF;
}

// This function is checking if a given string is contained in a file.
// It is working with strings containing multiple lines.
// This function is design to work on small files, it could be probably optimized
// by reading the file using loading by buffer.
bool file_contains_string(const char *filename, const char *str) {
  FILE *f = fopen(filename, "r");
  if (f == NULL)
    return false;

  int search_len = strlen(str);

  fseek(f, 0L, SEEK_END);
  int file_size = ftell(f);

  if (search_len > file_size) {
    fclose(f);
    return false;
  }

  fseek(f, 0L, SEEK_SET);

  char *file_content = (char *)malloc(file_size);
  int file_index = 0;

  int c;
  while ((c = fgetc(f)) != EOF) {
    file_content[file_index] = c;
    file_content[file_index + 1] = 0;
    if (file_index >= search_len) {
      if (strncmp(file_content + file_index - search_len, str, search_len) == 0) {
        free(file_content);
        fclose(f);
        return true;
      }
    }
    file_index++;
  }

  free(file_content);
  fclose(f);

  return false;
}

time_t file_get_creation_time(const char *filename) {
  struct stat attr;
  stat(filename, &attr);
  return attr.st_mtime;
}

#endif
