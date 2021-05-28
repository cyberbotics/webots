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

/*
 * Description:   Defines a packet
 */

#ifndef PACKET_HPP
#define PACKET_HPP

class Packet {
public:
  explicit Packet(int maxSize);
  virtual ~Packet();

  const unsigned char *data() const { return mData; }
  virtual void clear() { mIndex = 0; }
  int size() const { return mSize; }
  int maxSize() const { return mMaxSize; }
  void append(const unsigned char *data, int size);
  void append(const char *data, int size) { append((const unsigned char *)data, size); }
  void appendInt(int value);
  int readIntAt(int pos) const;
  const unsigned char *getBufferFromPos(int pos) const;
  bool readFromSocket(int socket, int n);

protected:
  int mMaxSize;
  int mSize;
  unsigned char *mData;
  int mIndex;

private:
  Packet(const Packet &);             // non constructor-copyable
  Packet &operator=(const Packet &);  // non copyable
};

#endif
