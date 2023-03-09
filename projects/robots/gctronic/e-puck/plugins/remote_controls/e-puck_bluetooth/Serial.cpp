// Copyright 1996-2023 Cyberbotics Ltd.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     https://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "Serial.hpp"

#include <fcntl.h>
#include <unistd.h>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <sstream>
#include <stdexcept>
#include <string>

#ifdef _WIN32
#include <ctype.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <winreg.h>

#else  // __linux__ || __APPLE__

#include <errno.h>
#include <termios.h>

#ifdef __APPLE__
#include <IOKit/IOBSD.h>
#include <IOKit/IOKitLib.h>
#include <IOKit/serial/IOSerialKeys.h>
#include <IOKit/serial/ioss.h>
#include <sys/param.h>

#else  // __linux__
#include <termio.h>

#endif
#endif

using namespace std;

vector<string> Serial::portNames;

#ifdef _WIN32
struct readFileThreadArguments {
  int nrd, c, size;
  char *buffer;
  HANDLE fd;
};
#endif

Serial::Serial(const string &port) : mName(port) {
#ifdef _WIN32
  DCB dcb;
  COMMTIMEOUTS commTimeouts;
#else  // __linux__ || __APPLE__
  struct termios term;
#endif

#ifdef _WIN32
  // cppcheck-suppress noCopyConstructor
  // cppcheck-suppress noOperatorEq
  mFd = CreateFile(mName.c_str(), GENERIC_READ | GENERIC_WRITE, 0, 0, OPEN_EXISTING, 0, 0);
  if (mFd == INVALID_HANDLE_VALUE) {
    mFd = (HANDLE)-1;
    throwFatalException("Cannot open serial port");
  }
  FillMemory(&dcb, sizeof(dcb), 0);
  if (!GetCommState(mFd, &dcb))
    throwFatalException("Unable to get serial attributes");

  dcb.DCBlength = sizeof(DCB);
  dcb.BaudRate = CBR_115200;
  dcb.ByteSize = 8;
  dcb.Parity = NOPARITY;
  dcb.StopBits = ONESTOPBIT;
  dcb.fBinary = true;
  dcb.fParity = NOPARITY;
  dcb.fAbortOnError = false;
  dcb.fRtsControl = RTS_CONTROL_DISABLE;
  dcb.fDtrControl = DTR_CONTROL_DISABLE;

  // Set new state
  if (!SetCommState(mFd, &dcb))
    throwFatalException("Unable to set serial attributes");

  // Fix Windows 2000 problem with laptops
  SetCommMask(mFd, EV_BREAK | EV_CTS | EV_DSR | EV_ERR | EV_RING | EV_RLSD | EV_RXCHAR | EV_RXFLAG | EV_TXEMPTY);

  // Change the COMMTIMEOUTS structure settings
  GetCommTimeouts(mFd, &commTimeouts);
  commTimeouts.ReadIntervalTimeout = 10;
  commTimeouts.ReadTotalTimeoutMultiplier = 10;
  commTimeouts.ReadTotalTimeoutConstant = 1000;
  commTimeouts.WriteTotalTimeoutMultiplier = 10;
  commTimeouts.WriteTotalTimeoutConstant = 1000;
  SetCommTimeouts(mFd, &commTimeouts);

#elif defined(__APPLE__)
  mFd = ::open(mName.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
  if (mFd == -1)
    throwFatalException("Could not open this port");

  if (fcntl(mFd, F_SETFL, 0) == -1) {
    close(mFd);
    throwFatalException("Failed to clear O_NDELAY");
  }

  // Get the current options
  if (tcgetattr(mFd, &term) == -1) {
    close(mFd);
    throwFatalException("Error getting the tty attributes");
  }

#else
  // __linux__
  int rval;

  // test device file existence
  rval = access(mName.c_str(), F_OK);
  if (rval != 0) {
    stringstream sstm;
    sstm << "File \"" << mName << "\" does not exists. " << strerror(errno);
    throwFatalException(sstm.str());
  }

  // test device file is readable
  rval = access(mName.c_str(), R_OK);
  if (rval != 0) {
    stringstream sstm;
    sstm << "File \"" << mName << "\" is not readable. " << strerror(errno);
    throwFatalException(sstm.str());
  }

  // test device file is writable
  rval = access(mName.c_str(), W_OK);
  if (rval != 0) {
    stringstream sstm;
    sstm << "File \"" << mName << "\" is not writable. " << strerror(errno);
    throwFatalException(sstm.str());
  }

  mFd = open(mName.c_str(), O_RDWR | O_NOCTTY);
  if (mFd < 0) {
    stringstream sstm;
    sstm << "File \"" << mName << "\" cannot be open. " << strerror(errno);
    throwFatalException(sstm.str());
  }

  tcflush(mFd, TCIOFLUSH);  // flush old data
  cfmakeraw(&term);         // switch to raw mode
  rval = tcsetattr(mFd, TCSANOW, &term);

  if (rval < 0) {
    mFd = -1;
    throwFatalException("Unable to set serial attributes");
  }
#endif  // __linux__
}

Serial::~Serial() {
#ifdef _WIN32
  if (mFd) {
    int ret = CloseHandle(mFd);
    if (ret == 0)
      throwFatalException("Error closing the serial communication");
  }
#else  // UNIX
  if (mFd > 0)
    ::close(mFd);
#endif
}

#ifdef _WIN32
DWORD Serial::readFileThread(void *param) {
  // ReadFile in a thread because it never returns if the commnication breaks suddenly
  readFileThreadArguments *ta = static_cast<readFileThreadArguments *>(param);
  ReadFile(ta->fd, &(ta->buffer[ta->nrd]), ta->size - ta->nrd, (LPDWORD) & (ta->c), 0);
  Sleep(1);
  return 0;
}
#endif

int Serial::read(char *packet, int size, bool wait) {
  int nrd = 0;

#ifdef _WIN32
  // Change the COMMTIMEOUTS structure settings
  COMMTIMEOUTS commTimeouts;
  GetCommTimeouts(mFd, &commTimeouts);
  commTimeouts.ReadIntervalTimeout = wait ? 10 : MAXDWORD;
  commTimeouts.ReadTotalTimeoutMultiplier = wait ? 10 : 0;
  commTimeouts.ReadTotalTimeoutConstant = wait ? 1000 : 0;
  SetCommTimeouts(mFd, &commTimeouts);

#else
  fd_set readfds;
  struct timeval timeout;
  FD_ZERO(&readfds);
  FD_SET(mFd, &readfds);
  timeout.tv_sec = wait ? 1 : 0;  // 1 second time out (to leave time to establish the connection)
  timeout.tv_usec = 0;
#endif

  do {
#ifdef _WIN32
    readFileThreadArguments ta;
    ta.nrd = nrd;
    ta.c = 0;
    ta.buffer = packet;
    ta.size = size;
    ta.fd = mFd;
    HANDLE rfThread = CreateThread(0, 0, (LPTHREAD_START_ROUTINE)readFileThread, &ta, 0, 0);
    // Break the thread after 1.2s of inactivity (ReadFile is crashed)
    if (WaitForSingleObject(rfThread, 1200) != 0) {
      DWORD exit = 0;
      TerminateThread(rfThread, exit);
      CloseHandle(rfThread);
      return -1;
    }
    CloseHandle(rfThread);
    nrd = ta.nrd;
    const int c = ta.c;
    if (nrd == 0 && c == 0)
      return -1;

#else
    if (select(mFd + 1, &readfds, 0, 0, &timeout) == 0)
      return nrd;
    const int c = ::read(mFd, &packet[nrd], size - nrd);
    if (c == -1)
      throwFatalException("Serial read error");
#endif

    nrd += c;
  } while (nrd != size && wait);
  return nrd;
}

void Serial::drain() {
  char buffer[1024];
  int bytesReaded;
  do {
    bytesReaded = this->read(buffer, 1024, true);
  } while (bytesReaded > 0);
}

void Serial::write(const char *packet, int size) {
#ifdef _WIN32
  int file_size = 0;
  WriteFile(mFd, packet, size, (LPDWORD)&file_size, 0);
#else
  if (::write(mFd, packet, size) == -1)
    throwFatalException("Serial write error");
  fsync(mFd);
#endif
  // printf("wrote %d bytes: %s\n",size==-1?strlen(text):size,size==-1?text:"binary");fflush(stdout);
}

char *Serial::readLine() {
  int size = 1024;
  char *buffer = static_cast<char *>(malloc(size));
  if (!buffer)
    return NULL;

  int pos = 0;
  bool success = true;
  char c, *pc;
  do {
    if (pos == size) {
      size += 1024;
      buffer = static_cast<char *>(realloc(buffer, size));
      if (!buffer) {
        fprintf(stderr, "Error reading from serial port: not enough memory.\n");
        exit(EXIT_FAILURE);
      }
    }

    pc = &buffer[pos++];
    int ret = this->read(pc, 1, true);
    if (ret != 1) {
      success = false;
      break;
    }
    c = *pc;
  } while (c != '\n' && c != '\0');

  if (success) {
    // strip terminating '\r\n' if any
    pc = strchr(buffer, '\n');
    if (pc)
      *pc = '\0';
    pc = strchr(buffer, '\r');
    if (pc)
      *pc = '\0';
  } else {
    free(buffer);
    buffer = NULL;
  }

  return buffer;
}

char *Serial::talk(const char *source) {
  write(source, strlen(source) + 1);

  return this->readLine();
}

void Serial::updatePorts() {
  portNames.clear();

#ifdef _WIN32
  static HKEY k;
  CHAR lpValueName[256];
  BYTE lpData[256];
  DWORD lpcValueName, lpcData;
  RegOpenKeyEx(HKEY_LOCAL_MACHINE, TEXT("HARDWARE\\DEVICEMAP\\SERIALCOMM"), 0, KEY_READ, &k);

  for (int i = 0;; i++) {
    lpcValueName = 256;
    lpcData = 256;
    if (RegEnumValue(k, i, lpValueName, &lpcValueName, 0, 0, lpData, &lpcData) == ERROR_SUCCESS) {
      // printf("found %ls: %ls\n",lpValueName,lpData);
      // lpData looks like "COM3" or "COM40"
      stringstream s;
      s << "\\\\.\\" << lpData;
      portNames.push_back(s.str());
      // according to the Windows API docs, the lpData string may not have been stored with the
      // proper null-terminating characters, hence we need to use strndup rather than strdup.
    } else
      break;
  }
  RegCloseKey(k);

#elif defined(__linux__)
  FILE *rfcomm = fopen("/etc/bluetooth/rfcomm.conf", "r");
  if (rfcomm) {
    char line[256];
    char filename[256];
    int inside = 0;
    while (fgets(line, 256, rfcomm)) {
      for (int i = 0; i < 256; i++) {
        if (line[i] == '#' || line[i] == '\n')
          break;
        else if (line[i] == ' ' || line[i] == '\t')
          continue;
        else if (line[i] == '{')
          inside++;
        else if (line[i] == '}')
          inside--;
        else if (inside == 0) {
          sscanf(&line[i], "%255s", filename);
          i += strlen(filename);
          stringstream s;
          s << "/dev/" << filename;
          portNames.push_back(s.str());
        }
      }
    }
    fclose(rfcomm);
  }

#elif defined(__APPLE__)
  CFMutableDictionaryRef classesToMatch;
  kern_return_t kernResult;
  io_iterator_t serialPortIterator;

  classesToMatch = IOServiceMatching(kIOSerialBSDServiceValue);
  if (classesToMatch == 0)
    printf("IOServiceMatching failed.\n");

  CFDictionarySetValue(classesToMatch, CFSTR(kIOSerialBSDTypeKey), CFSTR(kIOSerialBSDRS232Type));
  kernResult = IOServiceGetMatchingServices(kIOMasterPortDefault, classesToMatch, &serialPortIterator);
  if (kernResult != KERN_SUCCESS)
    printf("IOServiceGetMatchingServices failed: %d\n", kernResult);
  io_object_t service;
  while ((service = IOIteratorNext(serialPortIterator))) {
    CFTypeRef bsdPathAsCFString = IORegistryEntryCreateCFProperty(service, CFSTR(kIOCalloutDeviceKey), kCFAllocatorDefault, 0);
    if (bsdPathAsCFString) {
      Boolean result;
      char bsdPath[MAXPATHLEN];
      result = CFStringGetCString((CFStringRef)bsdPathAsCFString, bsdPath, sizeof(bsdPath), kCFStringEncodingUTF8);
      CFRelease(bsdPathAsCFString);
      if (result) {
        if (strstr(&bsdPath[8], "PDA-Sync") == 0 && strcmp(&bsdPath[8], "SerialPort-1") != 0) {
          portNames.push_back(string(bsdPath));
        }
      }
    }
    (void)IOObjectRelease(service);
  }
#endif
  for (int i = 0; i < 10; i++) {
    stringstream s;
    s << "WEBOTS_COM" << (i + 1);
    string name = s.str();
    const char *env = getenv(name.c_str());
    if (env && strlen(env) > 0)
      portNames.push_back(string(env));
  }
}

void Serial::throwFatalException(const string &errorMessage, bool displayLastError) const {
  stringstream s;
  s << "Serial error on port \"" << mName << "\" : " << errorMessage;
  if (displayLastError)
    s << endl << "Last error: " << strerror(errno);
  throw runtime_error(s.str());
}
