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

#include "Packet.hpp"

#include <sys/types.h>
#include <cassert>
#include <cstring>
#include <iostream>
#ifdef _WIN32
#include <winsock.h>
#else
#include <sys/socket.h>
#endif

using namespace std;

Packet::Packet(int maxSize) : mMaxSize(maxSize), mSize(0), mIndex(0) {
  mData = new unsigned char[mMaxSize];
}

Packet::~Packet() {
  delete[] mData;
}

void Packet::append(const unsigned char *data, int size) {
  assert(mIndex + size <= mMaxSize);
  memcpy(mData + mIndex, data, size);
  mIndex += size;
  mSize += size;
}

void Packet::appendInt(int value) {
  unsigned char array[4];
  array[0] = (value >> 24) & 0xff;
  array[1] = (value >> 16) & 0xff;
  array[2] = (value >> 8) & 0xff;
  array[3] = value & 0xff;
  append(array, 4);
}

int Packet::readIntAt(int pos) const {
  assert(pos + 3 < mSize);
  unsigned char c1 = mData[pos + 3];
  unsigned char c2 = mData[pos + 2];
  unsigned char c3 = mData[pos + 1];
  unsigned char c4 = mData[pos];
  int r = c1 + (c2 << 8) + (c3 << 16) + (c4 << 24);
  return r;
}

const unsigned char *Packet::getBufferFromPos(int pos) const {
  assert(pos < mSize);
  return &mData[pos];
}

bool Packet::readFromSocket(int socket, int n) {
  n += mIndex;
  assert(n <= mMaxSize);
  do {
    int r = recv(socket, (char *)&mData[mIndex], n - mIndex, 0);
    if (r == -1)
      return false;
    mIndex += r;
  } while (mIndex < n);
  mSize += n;
  return true;
}
