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

#ifdef _WIN32
#include <windows.h>
#else  // __linux__ || __APPLE__
#include <dlfcn.h>
#endif

#include <stdio.h>
#include <string.h>
#include <webots/utils/system.h>
#include "dynamic_library.h"

#ifdef _WIN32
#define DYNAMIC_LIBRARY_LOAD(a) LoadLibraryEx(wbu_system_short_path(a), NULL, LOAD_WITH_ALTERED_SEARCH_PATH)
#define DYNAMIC_LIBRARY_GETSYM(a, b) GetProcAddress(a, b)
#define DYNAMIC_LIBRARY_UNLOAD(a) FreeLibrary(a)

#else  // __linux__ || __APPLE__
#define DYNAMIC_LIBRARY_LOAD(a) dlopen(a, RTLD_LAZY | RTLD_GLOBAL)
#define DYNAMIC_LIBRARY_GETSYM(a, b) dlsym(a, b)
#define DYNAMIC_LIBRARY_UNLOAD(a) dlclose(a)
#endif

static void dynamic_library_print_last_error() {
#ifdef _WIN32
  LPVOID lpMsgBuf = NULL;
  FormatMessage(FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_IGNORE_INSERTS, NULL,
                GetLastError(), MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT), (LPTSTR)&lpMsgBuf, 0, NULL);
  // cppcheck-suppress nullPointer
  fprintf(stderr, "Error: %s (dynamic library)\n", (const char *)lpMsgBuf);
  LocalFree(lpMsgBuf);
#else
  const char *s = dlerror();
  if (!s)
    fprintf(stderr, "Error: unknown error in dynamic library\n");
  else
    fprintf(stderr, "Error: %s (dynamic library)\n", s);
#endif
}

DYNAMIC_LIBRARY_HANDLE dynamic_library_init(const char *libname) {
  const int length = strlen(libname);
  if (length == 0)
    return NULL;
  char *tmpname = malloc(length + 4);
  if (!tmpname)
    return NULL;
  memcpy(tmpname, libname, length);
  tmpname[length] = '\0';
#ifdef __linux__
  // dlopen() does not add .so to the filename, like windows does for .dll
  if (length > 3 && (libname[length - 3] != '.' || libname[length - 2] != 's' || libname[length - 1] != 'o'))
    strcat(tmpname, ".so");
#endif
  DYNAMIC_LIBRARY_HANDLE lib = (DYNAMIC_LIBRARY_HANDLE)DYNAMIC_LIBRARY_LOAD(tmpname);
  if (!lib)
    dynamic_library_print_last_error();
  free(tmpname);
  return lib;
}

void dynamic_library_cleanup(DYNAMIC_LIBRARY_HANDLE lib) {
  if (lib)
    DYNAMIC_LIBRARY_UNLOAD(lib);
}

void *dynamic_library_get_symbol(DYNAMIC_LIBRARY_HANDLE lib, const char *symbol) {
  if (!lib)
    return NULL;
  return (void *)DYNAMIC_LIBRARY_GETSYM(lib, symbol);
}
