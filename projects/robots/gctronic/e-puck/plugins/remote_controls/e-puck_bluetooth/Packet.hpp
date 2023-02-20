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
 * Description:  Defines a packet
 */

#ifndef PACKET_HPP
#define PACKET_HPP

class Packet {
public:
  explicit Packet(int maxSize, int initSize = 0);
  virtual ~Packet();

  char *data() const { return mData; }

  virtual void clear() { mSize = 0; }
  int size() const { return mSize; }

  void append(char v);
  void append(short i);

  short readShortAt(int pos) const;
  unsigned short readUShortAt(int pos) const;
  unsigned char readUCharAt(int pos) const;

  void print();

private:
  Packet(const Packet &);             // non constructor-copyable
  Packet &operator=(const Packet &);  // non copyable
  int mMaxSize;
  int mSize;
  char *mData;
};

#endif
