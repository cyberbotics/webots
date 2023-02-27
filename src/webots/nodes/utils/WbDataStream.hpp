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

#ifndef WB_DATA_STREAM_HPP
#define WB_DATA_STREAM_HPP

#include <QtCore/QByteArray>

class QByteArray;

class WbDataStream : public QByteArray {
public:
  using QByteArray::QByteArray;

  int mSizePtr;
  int mDataSize;

  WbDataStream &writeRawData(const char *s, int len);

  WbDataStream &operator<<(qint8 n);
  WbDataStream &operator<<(quint8 n);
  WbDataStream &operator<<(qint16 n);
  WbDataStream &operator<<(quint16 n);
  WbDataStream &operator<<(qint32 n);
  WbDataStream &operator<<(quint32 n);
  WbDataStream &operator<<(qint64 n);
  WbDataStream &operator<<(quint64 n);

  WbDataStream &operator<<(bool n);
  WbDataStream &operator<<(float f);
  WbDataStream &operator<<(double f);
  WbDataStream &operator<<(const char *s);

  void increaseNbChunks(unsigned short n);
};

inline WbDataStream &WbDataStream::operator<<(quint8 n) {
  return *this << qint8(n);
}
inline WbDataStream &WbDataStream::operator<<(quint16 n) {
  return *this << qint16(n);
}
inline WbDataStream &WbDataStream::operator<<(quint32 n) {
  return *this << qint32(n);
}
inline WbDataStream &WbDataStream::operator<<(quint64 n) {
  return *this << qint64(n);
}

#endif
