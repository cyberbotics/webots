/*
 * Copyright 1996-2021 Cyberbotics Ltd.
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
    // cppcheck-suppress nullPointer
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
    l += 3 + strlen(argv[i]);          // spaces and double quotes between arguments
  char *command_line = malloc(l + 7);  // room for double quotes, the extra "-bin" string and final '\0'
  command_line[0] = '\"';
  command_line[1] = '\0';
  strcat(command_line, module_path);
  command_line[l0 - 3] = '\0';  // cut out ".exe" after "webots" or "webotsw"
  strcat(command_line, "-bin.exe");
  strcat(command_line, "\"");
  for (int i = 1; i < argc; i++) {
    strcat(command_line, " \"");
    strcat(command_line, argv[i]);
    strcat(command_line, "\"");
  }

  // add "WEBOTS_HOME/msys64/mingw64/bin", "WEBOTS_HOME/msys64/mingw64/bin/cpp" and "WEBOTS_HOME/msys64/usr/bin" to the PATH
  // environment variable
  char *old_path = malloc(LENGTH);
  char *new_path = malloc(LENGTH);

  strcpy(new_path, module_path);
  new_path[l0 - 11] = ';';  // removes "\webots.exe" or "\webotsw.exe"
  strcpy(&new_path[l0 - 10], module_path);
  new_path[2 * l0 - 21] = '\0';
  strcat(new_path, "\\cpp;");
  strcat(new_path, module_path);
  free(module_path);
  new_path[3 * l0 - 38] = '\0';
  strcat(new_path, "usr\\bin;");
  if (!GetEnvironmentVariable("PATH", old_path, LENGTH))
    fail("GetEnvironmentVariable", 0);
  strcat(new_path, old_path);
  free(old_path);
  if (!SetEnvironmentVariable("PATH", new_path))
    fail("SetEnvironmentVariable", new_path);
  free(new_path);
  if (!SetEnvironmentVariable("QT_ENABLE_HIGHDPI_SCALING", "1"))
    fail("SetEnvironmentVariable", "QT_ENABLE_HIGHDPI_SCALING=1");

  // start the webots-bin.exe process, wait for completion and return exit code
  STARTUPINFO info = {sizeof(info)};
  PROCESS_INFORMATION process_info;
  if (!CreateProcess(NULL, command_line, NULL, NULL, TRUE, 0, NULL, NULL, &info, &process_info))
    fail("CreateProcess", command_line);
  free(command_line);

  // webots-bin.exe should be killed whenever its parent (webots.exe or webotsw.exe) terminates.
  HANDLE job = CreateJobObject(NULL, NULL);
  JOBOBJECT_EXTENDED_LIMIT_INFORMATION jeli = {0};
  jeli.BasicLimitInformation.LimitFlags = JOB_OBJECT_LIMIT_KILL_ON_JOB_CLOSE;
  if (!SetInformationJobObject(job, JobObjectExtendedLimitInformation, &jeli, sizeof(jeli)))
    fail("SetInformationJobObject", 0);
  if (!AssignProcessToJobObject(job, process_info.hProcess))
    fail("AssignProcessToJobObject", 0);

  // wait for webots-bin.exe to terminate
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
