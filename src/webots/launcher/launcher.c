/*
 * Copyright 1996-2020 Cyberbotics Ltd.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

// Description:  This is the source code of webots.exe and webotsw.exe, the binary launchers of webots-bin.exe (Windows only).
//               It adds paths to the PATH environment variable to ensure that the correct libraries will be found and starts
//               webots-bin.exe, passing all the command line arguments, wait until the completion of webots-bin.exe and
//               returns the exit status code of webots-bin.exe.
//               The main advantage of a binary launcher over a batch launcher (webots.bat) is that the binary launcher
//               doesn't open a DOS cmd.exe console in the background, whereas this is unavoidable when running a batch file.
//               webotsw.exe is a windows application (to be started from the icon or menu).
//               webots.exe is DOS application (to be started from a DOS command prompt or a script).
//               Starting webots.exe from the icon is fine, but will open a DOS command prompt in the background.
//               (a similar naming convention is used for python.exe / pythonw.exe, java.exe / javaw.exe, etc.)

#include <stdio.h>
#include <windows.h>

static int fail(const char *function, const char *info) {
  DWORD e = GetLastError();
  if (e) {
    LPSTR m = NULL;
    FormatMessage(FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_IGNORE_INSERTS, NULL, e,
                  MAKELANGID(LANG_ENGLISH, SUBLANG_ENGLISH_US), (LPSTR)&m, 0, NULL);
    char message[1024];
    const char *lf = info ? "\n" : "";
    const char *i = info ? info : "";
    snprintf(message, sizeof(message), "%s failed with error %lu.\n%s%s%s", function, e, m, i, lf);
    LocalFree(m);
#ifdef WEBOTSW
    MessageBox(NULL, message, "Webots launcher error", MB_ICONERROR | MB_OK);
#else
    fprintf(stderr, message);
#endif
  } else
    fprintf(stderr, "%s failed with no error.\n", function);
  exit(e);
}

int main(int argc, char *argv[]) {
  // compute the full command line with absolute path for webots-bin.exe, options and arguments
  const int LENGTH = 4096;
  char *module_path = malloc(LENGTH);
  if (!GetModuleFileName(NULL, module_path, LENGTH))
    fail("GetModuleFileName", 0);
  int l = strlen(module_path);
#ifdef WEBOTSW
  const int l0 = l - 1;  // webotsw.exe (we need to remove the final 'w')
#else
  const int l0 = l;  // webots.exe
#endif
  l = l0;
  for (int i = 1; i < argc; i++)
    l += 1 + strlen(argv[i]);          // spaces between arguments
  char *command_line = malloc(l + 5);  // room for the extra "-bin" string and final '\0'
  command_line[0] = '\0';              // initially empty string
  strcat(command_line, module_path);
  command_line[l0 - 4] = '\0';  // cut out ".exe" after "webots" or "webotsw"
  strcat(command_line, "-bin.exe");
  for (int i = 1; i < argc; i++) {
    strcat(command_line, " ");
    strcat(command_line, argv[i]);
  }

  // add "WEBOTS_HOME/msys64/mingw64/bin" and "WEBOTS_HOME/msys64/usr/bin" to the PATH environment variable
  char *old_path = malloc(LENGTH);
  char *new_path = malloc(LENGTH);

  strncpy(new_path, module_path, l0);
  new_path[l0 - 11] = ';';  // removes "\webots.exe" or "\webotsw.exe"
  strncpy(&new_path[l0 - 10], module_path, l0);
  free(module_path);
  new_path[2 * l0 - 32] = '\0';  // add the final '\0'
  strcat(new_path, "usr\\bin;");
  if (!GetEnvironmentVariable("PATH", old_path, LENGTH))
    fail("GetEnvironmentVariable", 0);
  strcat(new_path, old_path);
  free(old_path);
  if (!SetEnvironmentVariable("PATH", new_path))
    fail("SetEnvironmentVariable", new_path);
  free(new_path);

  // start the webots-bin.exe process, wait for completion and return exit code
  STARTUPINFO info = {sizeof(info)};
  PROCESS_INFORMATION process_info;
  if (!CreateProcess(NULL, command_line, NULL, NULL, TRUE, 0, NULL, NULL, &info, &process_info))
    fail("CreateProcess", command_line);
  free(command_line);
  WaitForSingleObject(process_info.hProcess, INFINITE);  // return zero in case of success
  DWORD exit_code;
  if (!GetExitCodeProcess(process_info.hProcess, &exit_code))
    fail("GetExitCodeProcess", 0);
  if (!CloseHandle(process_info.hProcess))
    fail("CloseHandle", 0);
  if (!CloseHandle(process_info.hThread))
    fail("CloseHandle", 0);
  return exit_code;
}
