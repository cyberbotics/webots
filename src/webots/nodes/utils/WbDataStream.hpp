// Copyright 1996-2022 Cyberbotics Ltd.
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

#ifndef WB_DATA_STREAM_HPP
#define WB_DATA_STREAM_HPP

#include <QtCore/QByteArray>
#include <QtCore/QFloat16>

class qfloat16;
class QByteArray;

class WbDataStream : public QByteArray {
public:
  using QByteArray::QByteArray;

  int size_ptr = 0;
  int data_size = 0;

  WbDataStream &writeRawData(const char *s, int len);

  WbDataStream &operator<<(qint8 i);
  WbDataStream &operator<<(quint8 i);
  WbDataStream &operator<<(qint16 i);
  WbDataStream &operator<<(quint16 i);
  WbDataStream &operator<<(qint32 i);
  WbDataStream &operator<<(quint32 i);
  WbDataStream &operator<<(qint64 i);
  WbDataStream &operator<<(quint64 i);

  WbDataStream &operator<<(bool i);
  WbDataStream &operator<<(float f);
  WbDataStream &operator<<(double f);
  // WbDataStream &operator<<(qfloat16 f);
  WbDataStream &operator<<(const char *s);
};

inline WbDataStream &WbDataStream::operator<<(quint8 i) {
  return *this << qint8(i);
}
inline WbDataStream &WbDataStream::operator<<(quint16 i) {
  return *this << qint16(i);
}
inline WbDataStream &WbDataStream::operator<<(quint32 i) {
  return *this << qint32(i);
}
inline WbDataStream &WbDataStream::operator<<(quint64 i) {
  return *this << qint64(i);
}

#endif
