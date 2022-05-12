/*
 * Copyright 1996-2022 Cyberbotics Ltd.
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

#include <stdio.h>
#include <string.h>

#ifdef _WIN32
#include <winsock2.h>
#else
#include <arpa/inet.h>
#include <netdb.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>
#include <webots/types.h>

#endif

#include "tcp_client.h"

TcpClient *tcp_client_new(const char *host, int port) {
  TcpClient *c = malloc(sizeof(TcpClient));

  c->fd = tcp_client_open();

  if (c->fd < 0) {
    fprintf(stderr, "socket() failed\n");
    // cppcheck-suppress memleak ; otherwise cppcheck shows a false positive for p->handle
    free(c);
    return NULL;
  }

  int connect = tcp_client_connect(c, host, port);

  if (connect == -1 || connect == 0) {  // Failed to lookup host or connection failed
    tcp_client_close(c);
    return NULL;
  }

  return c;
}

int tcp_client_open() {
  int fd;
#ifdef _WIN32
  // initialize the socket API if needed
  WSADATA info;
  if (WSAStartup(MAKEWORD(1, 1), &info) != 0) {  // Winsock 1.1
    fprintf(stderr, "Cannot initialize Winsock.\n");
    return -1;
  }
#endif

  /* create the socket */
  fd = socket(AF_INET, SOCK_STREAM, 0);
  if (fd == -1) {
    fprintf(stderr, "Cannot create socket.\n");
    return -1;
  }
  return fd;
}

int tcp_client_connect(TcpClient *c, const char *host, int port) {
  struct sockaddr_in address;
  struct hostent *server;
  // fill in the socket address
  memset(&address, 0, sizeof(struct sockaddr_in));
  address.sin_family = AF_INET;
  address.sin_port = htons(port);
  server = gethostbyname(host);

  if (server)
    memcpy((char *)&address.sin_addr.s_addr, (char *)server->h_addr, server->h_length);
  else {
    fprintf(stderr, "Cannot resolve server name: %s.\n", host);
    return -1;
  }
  /* connect to the server */
  int rc = connect(c->fd, (struct sockaddr *)&address, sizeof(struct sockaddr));
  if (rc == -1)
    return 0;
  return 1;
}

bool tcp_client_send(TcpClient *c, const char *buffer, int size) {
  char *p = (char *)buffer;
  while (size > 0) {
    int i = send(c->fd, p, size, 0);
    if (i < 1)
      return false;
    p += i;
    size -= i;
  }
  return true;
}

int tcp_client_receive(TcpClient *c, char *buffer, int size) {
  return recv(c->fd, buffer, size, 0);
}

void tcp_client_close(TcpClient *c) {
#ifdef _WIN32
  closesocket(c->fd);
  free(c);
  if (WSACleanup() != 0)
    fprintf(stderr, "Cannot cleanup Winsock.\n");
#else
  close(c->fd);
  free(c);
#endif
}
