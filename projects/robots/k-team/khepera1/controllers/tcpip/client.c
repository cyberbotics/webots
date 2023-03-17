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

/*
 * Description:  A simple client program to connect to the TCP/IP server thanks to Darren Smith
 */

/*
 * Linux:   compile with gcc -Wall client.c -o client
 * Windows: compile with gcc -Wall -mno-cygwin client.c -o client -lws2_32
 */

#include <stdio.h>
#include <string.h>

#ifdef _WIN32
#include <winsock.h>
#else
#include <arpa/inet.h>
#include <netdb.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>
#endif

#define SOCKET_PORT 10020
#define SOCKET_SERVER "127.0.0.1" /* local host */

int main(int argc, char *argv[]) {
  struct sockaddr_in address;
  struct hostent *server;
  int fd, rc;
  char buffer[256];

#ifdef _WIN32
  /* initialize the socket api */
  WSADATA info;

  rc = WSAStartup(MAKEWORD(1, 1), &info); /* Winsock 1.1 */
  if (rc != 0) {
    printf("cannot initialize Winsock\n");

    return -1;
  }
#endif
  /* create the socket */
  fd = socket(AF_INET, SOCK_STREAM, 0);
  if (fd == -1) {
    printf("cannot create socket\n");
    return -1;
  }

  /* fill in the socket address */
  memset(&address, 0, sizeof(struct sockaddr_in));
  address.sin_family = AF_INET;
  address.sin_port = htons(SOCKET_PORT);
  server = gethostbyname(SOCKET_SERVER);

  if (server)
    memcpy((char *)&address.sin_addr.s_addr, (char *)server->h_addr, server->h_length);
  else {
    printf("cannot resolve server name: %s\n", SOCKET_SERVER);
#ifdef _WIN32
    closesocket(fd);
#else
    close(fd);
#endif
    return -1;
  }

  /* connect to the server */
  rc = connect(fd, (struct sockaddr *)&address, sizeof(struct sockaddr));
  if (rc == -1) {
    printf("cannot connect to the server\n");
#ifdef _WIN32
    closesocket(fd);
#else
    close(fd);
#endif
    return -1;
  }

  for (;;) {
    printf("Enter command: ");
    fflush(stdout);
    scanf("%255s", buffer);
    int n = strlen(buffer);
    buffer[n++] = '\n'; /* append carriage return */
    buffer[n] = '\0';
    n = send(fd, buffer, n, 0);

    if (strncmp(buffer, "exit", 4) == 0)
      break;

    n = recv(fd, buffer, 256, 0);
    buffer[n] = '\0';
    printf("Answer is: %s", buffer);
  }

#ifdef _WIN32
  closesocket(fd);
#else
  close(fd);
#endif

  return 0;
}
