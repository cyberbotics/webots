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

#include "Communication.hpp"
#include "Packet.hpp"
#include "Time.hpp"

#include <iostream>
#include <sstream>
#include <stdexcept>

#include <cstdlib>
#include <cstring>

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

using namespace std;

Communication::Communication() : mSocket(-1) {
#ifdef _WIN32
  // initialize the socket API
  WSADATA info;
  int rc = WSAStartup(MAKEWORD(1, 1), &info);  // Winsock 1.1
  if (rc != 0)
    cerr << "Cannot initialize Winsock" << endl;
#endif
}

Communication::~Communication() {
  close();
#ifdef _WIN32
  WSACleanup();
#endif
}

bool Communication::initialize(const char *ip, int port) {
  close();
  mSocket = socket(AF_INET, SOCK_STREAM, 0);
  if (mSocket == -1) {
    cerr << "Cannot create socket" << endl;
    return false;
  }
  struct sockaddr_in address;
  memset(&address, 0, sizeof(struct sockaddr_in));  // fill in the socket address
  address.sin_family = AF_INET;
  address.sin_port = htons(port);
  struct hostent *server = gethostbyname(ip);
  if (server)
    memcpy((char *)&address.sin_addr.s_addr, (char *)server->h_addr, server->h_length);
  else {
    cerr << "Cannot resolve server name: " << ip << endl;
    return false;
  }
  // connect to the server
  int rc = connect(mSocket, (struct sockaddr *)&address, sizeof(struct sockaddr));
  if (rc == -1) {
    close();
    return false;
  }
  return true;
}

void Communication::close() {
  if (mSocket == -1)
    return;
#ifdef _WIN32
  closesocket(mSocket);
#else
  ::close(mSocket);
#endif
  mSocket = -1;
}

bool Communication::sendPacket(const Packet *packet) {
  if (mSocket == -1) {
    cerr << "Socket not initialized" << endl;
    return false;
  }
  int n = 0;
  int size = packet->size();
  do {
    int s = send(mSocket, (const char *)(packet->getBufferFromPos(n)), size - n, 0);
    if (s == -1) {
      cerr << "Error sending data to socket" << endl;
      return false;
    }
    n += s;
  } while (n < size);
  return true;
}

bool Communication::receivePacket(Packet *packet) {
  unsigned char buffer[5];
  int n = 0;
  do {  // read until the initial 'W' message
    n = recv(mSocket, (char *)buffer, 1, 0);
    if (n == -1) {
      cerr << "Error received packet" << endl;
      return false;
    }
  } while (buffer[0] != 'W');
  do {  // read the message size (int)
    int r = recv(mSocket, (char *)&buffer[n], 5 - n, 0);
    if (r == -1) {
      cerr << "Error received packet" << endl;
      return false;
    }
    n += r;
  } while (n < 5);
  packet->clear();
  packet->append(buffer, 5);
  int packet_size = packet->readIntAt(1);
  if (packet_size > packet->maxSize()) {
    cerr << "Too big packet about to be received" << endl;
    return false;
  }
  if (packet_size > 5)
    packet->readFromSocket(mSocket, packet_size - 5);
  return true;
}
