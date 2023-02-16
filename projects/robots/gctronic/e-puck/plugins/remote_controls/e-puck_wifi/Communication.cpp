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

/*
 * Description:  Implementation of Communication.hpp functions
 */

#include "Communication.hpp"

#include "EPuckCommandPacket.hpp"

#include <iostream>
#include <sstream>
#include <stdexcept>

#include <cstdlib>
#include <cstring>

#ifdef _WIN32
#include <winsock.h>
#else
#include <arpa/inet.h>
#include <errno.h>
#include <netdb.h>
#include <netinet/in.h>
#include <stdio.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>
#endif

using namespace std;

#define SOCKET_PORT 1000

Communication::Communication() : mFd(0) {
#ifdef _WIN32  // initialize the socket api
  WSADATA info;
  int rc = WSAStartup(MAKEWORD(1, 1), &info);  // Winsock 1.1
  if (rc != 0) {
    fprintf(stderr, "Cannot initialize Winsock\n");
  }
#endif
}

Communication::~Communication() {
  cleanup();
}

bool Communication::initialize(const string &ip) {
  struct sockaddr_in address;
  struct hostent *server;
  int rc;
  mFd = socket(AF_INET, SOCK_STREAM, 0);
  if (mFd == -1) {
    fprintf(stderr, "Cannot create socket\n");
    return false;
  }
  memset(&address, 0, sizeof(struct sockaddr_in));
  address.sin_family = AF_INET;
  address.sin_port = htons(SOCKET_PORT);
  server = gethostbyname(ip.c_str());
  if (server)
    memcpy(reinterpret_cast<char *>(&address.sin_addr.s_addr), reinterpret_cast<char *>(server->h_addr), server->h_length);
  else {
    fprintf(stderr, "Cannot resolve server name: %s\n", ip.c_str());
    cleanup();
    return false;
  }
  rc = connect(mFd, (struct sockaddr *)&address, sizeof(struct sockaddr));
  if (rc == -1) {
    fprintf(stderr, "Cannot connect to the server\n");
    cleanup();
    return false;
  }
  return true;
}

void Communication::cleanup() {
  if (mFd > 0)
#ifdef _WIN32
    closesocket(mFd);
#else
    close(mFd);
#endif
  mFd = 0;
}

bool Communication::send(const char *data, int size) {
  int n = 0;
  do {
    int m = ::send(mFd, &data[n], size - n, 0);
    if (m == -1)
      return false;
    n += m;
  } while (n < size);
  return n == size;
}

int Communication::receive(char *data, int size, bool block) {
  int n = 0;
  int flag;

#ifdef _WIN32

  if (!block) {
    u_long iMode = 0;
    if (ioctlsocket(mFd, FIONBIO, &iMode) != NO_ERROR)
      fprintf(stderr, "ioctlsocket failed\n");
  }
  flag = 0;

#else

  flag = block ? 0 : MSG_DONTWAIT;

#endif

  do {
    int m = ::recv(mFd, &data[n], size - n, flag);
#ifdef _WIN32
    if (m == SOCKET_ERROR) {
      if (WSAGetLastError() == WSAEWOULDBLOCK)
        return 0;
      else
        return -1;
    }
#else
    if (m == -1) {
      if (errno == EAGAIN || errno == EWOULDBLOCK)
        return 0;
      else
        return -1;
    }
#endif
    n += m;
  } while (n < size);
  return size;  // success
}
