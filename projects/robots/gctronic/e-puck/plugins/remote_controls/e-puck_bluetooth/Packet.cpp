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

#include "Packet.hpp"

#include <stdlib.h>
#include <iomanip>
#include <iostream>
#include <stdexcept>

using namespace std;

Packet::Packet(int maxSize, int initSize) : mMaxSize(maxSize), mSize(initSize) {
  mData = static_cast<char *>(malloc(maxSize));
  if (!mData)
    throw runtime_error("Malloc failed");
}

Packet::~Packet() {
  free(mData);
}

void Packet::append(char v) {
  if (mSize + (int)sizeof(char) >= mMaxSize)
    throw overflow_error("Packet max size overflow");

  mData[mSize++] = v;
}

void Packet::append(short i) {
  if (mSize + (int)sizeof(short) >= mMaxSize)
    throw overflow_error("Packet max size overflow");

  mData[mSize++] = i & 0xFF;
  mData[mSize++] = i >> 8;
}

unsigned short Packet::readUShortAt(int pos) const {
  if (pos + (int)sizeof(unsigned short) > mMaxSize)
    throw overflow_error("Packet max size overflow");

  unsigned char c1 = static_cast<unsigned char>(mData[pos]);
  unsigned char c2 = static_cast<unsigned char>(mData[pos + 1]);
  return c1 + (c2 << 8);
}

short Packet::readShortAt(int pos) const {
  if (pos + (int)sizeof(short) > mMaxSize)
    throw overflow_error("Packet max size overflow");

  unsigned char c1 = static_cast<unsigned char>(mData[pos]);
  unsigned char c2 = static_cast<unsigned char>(mData[pos + 1]);
  return c1 + (c2 << 8);
}

unsigned char Packet::readUCharAt(int pos) const {
  if (pos + (int)sizeof(unsigned char) > mMaxSize)
    throw overflow_error("Packet max size overflow");

  return static_cast<unsigned char>(mData[pos]);
}

void Packet::print() {
  cout << "Content:" << endl;
  for (int i = 0; i < mSize; i++) {
    cout << setw(6) << i << ": ";
    cout << uppercase << setw(2) << setfill('0') << hex << (((int)mData[i]) & 0xFF);
    if (-mData[i] >= 'A' && -mData[i] <= 'Z')
      cout << " (" << (char)-mData[i] << ")";
    cout << nouppercase << dec << setw(0) << setfill(' ');  // restore cout
    cout << endl;
  }
}
