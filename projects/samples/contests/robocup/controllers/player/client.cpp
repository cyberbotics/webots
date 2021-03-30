// Copyright 1996-2021 Cyberbotics Ltd.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <stdio.h>
#include <string.h>
#include <iostream>

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

#include <google/protobuf/io/zero_copy_stream_impl.h>
#include <google/protobuf/text_format.h>
#include "messages.pb.h"
#if GOOGLE_PROTOBUF_VERSION < 3006001
#define ByteSizeLong ByteSize
#endif

static void close_socket(int fd) {
#ifdef _WIN32
  closesocket(fd);
#else
  close(fd);
#endif
}

static char *read_file(const char *filename) {
  char *buffer = NULL;
  FILE *fp = fopen(filename, "r");
  if (!fp)
    return NULL;
  if (fseek(fp, 0L, SEEK_END) == 0) {
    const long size = ftell(fp);
    assert(size != -1);
    buffer = (char *)malloc(sizeof(char) * (size + 1));
    fseek(fp, 0L, SEEK_SET);
    const size_t len = fread(buffer, sizeof(char), size, fp);
    buffer[len] = '\0';
  }
  fclose(fp);
  return buffer;
}

static void socket_closed_exit() {
  printf("Connection closed by server.\n");
  exit(1);
}

int main(int argc, char *argv[]) {
  struct sockaddr_in address;
  struct hostent *server;
  int fd, rc;
  char *buffer;
  int port = 10003;
  char host[256];  // localhost

  GOOGLE_PROTOBUF_VERIFY_VERSION;

  sprintf(host, "127.0.0.1");
  if (argc > 1) {
    if (strcmp(argv[1], "--help") == 0 || strcmp(argv[1], "-h") == 0) {
      printf("Usage: client <IP:port> or <IP> or <port>\n");
      return 0;
    }
    const char *n = strchr(argv[1], ':');
    if (n > 0) {
      port = atoi(n + 1);
      strncpy(host, argv[1], sizeof(host) - 1);
      host[n - argv[1]] = '\0';
    } else if (strchr(argv[1], '.') || !isdigit(argv[1][0]))
      strncpy(host, argv[1], sizeof(host) - 1);
    else
      port = atoi(argv[1]);
  }
#ifdef _WIN32
  WSADATA info;
  rc = WSAStartup(MAKEWORD(2, 2), &info);  // Winsock 2.2
  if (rc != 0) {
    fprintf(stderr, "Cannot initialize Winsock\n");
    return 1;
  }
#endif
  fd = socket(AF_INET, SOCK_STREAM, 0);
  if (fd == -1) {
    fprintf(stderr, "Cannot create socket\n");
    return 1;
  }
  memset(&address, 0, sizeof(struct sockaddr_in));
  address.sin_family = AF_INET;
  address.sin_port = htons(port);
  server = gethostbyname(host);

  if (server)
    memcpy((char *)&address.sin_addr.s_addr, (char *)server->h_addr, server->h_length);
  else {
    fprintf(stderr, "Cannot resolve server name: %s\n", host);
    close_socket(fd);
    return 1;
  }
  rc = connect(fd, (struct sockaddr *)&address, sizeof(struct sockaddr));
  if (rc == -1) {
    fprintf(stderr, "Cannot connect to %s:%d\n", host, port);
    close_socket(fd);
    return 1;
  }
  char answer[8];
  int n = recv(fd, answer, sizeof(answer), 0);
  if (n > 0) {
    if (strncmp(answer, "Welcome", 8) != 0) {
      if (strncmp(answer, "Refused", 8) == 0)
        printf("Connection to %s:%d refused: your IP address is not allowed in the game.json configuration file.\n", host,
               port);
      else
        printf("Received unknown answer from server: %s\n", answer);
      return 1;
    }
  } else {
    printf("Connection closed.\n");
    return 1;
  }
  printf("Connected to %s:%d\n", host, port);
  for (;;) {
    const char *message = read_file("actuator_requests.txt");
    printf("Message = %s\n", message);

    ActuatorRequests actuatorRequests;
    google::protobuf::TextFormat::ParseFromString(message, &actuatorRequests);
#ifndef _WIN32
    // This doesn't work on Windows, we should implement SocketOutputStream to make it work efficiently on Windows
    // See https://stackoverflow.com/questions/23280457/c-google-protocol-buffers-open-http-socket
    const int size = htonl(actuatorRequests.ByteSizeLong());
    if (send(fd, (char *)(&size), sizeof(int), 0) == -1)
      socket_closed_exit();
    google::protobuf::io::ZeroCopyOutputStream *zeroCopyStream = new google::protobuf::io::FileOutputStream(fd);
    actuatorRequests.SerializeToZeroCopyStream(zeroCopyStream);
    delete zeroCopyStream;
#else  // here we make a useless malloc, copy and free
    const int size = actuatorRequests.ByteSizeLong();
    char *output = (char *)malloc(sizeof(int) + size);
    int *output_size = (int *)output;
    *output_size = htonl(size);
    actuatorRequests.SerializeToArray(&output[sizeof(int)], size);
    if (send(fd, output, sizeof(int) + size, 0) == -1) {
      free(output);
      socket_closed_exit();
    }
    free(output);
#endif
    int s;
    if (recv(fd, (char *)&s, sizeof(int), 0) == -1)
      socket_closed_exit();
    const int answer_size = ntohl(s);
    SensorMeasurements sensorMeasurements;
    if (answer_size) {
      buffer = (char *)malloc(answer_size);
      int i = 0;
      while (i < answer_size) {
        n = recv(fd, &buffer[i], answer_size, 0);
        if (n == -1)
          socket_closed_exit();
        i += n;
      }
      sensorMeasurements.ParseFromArray(buffer, answer_size);
      free(buffer);
    }
    std::string printout;
    google::protobuf::TextFormat::PrintToString(sensorMeasurements, &printout);
    std::cout << printout << std::endl;
  }
  close_socket(fd);
  printf("Connection closed.\n");
  return 0;
}
