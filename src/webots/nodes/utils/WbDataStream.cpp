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

#include "WbDataStream.hpp"

WbDataStream &WbDataStream::operator<<(qint8 i) {
  return (WbDataStream &)append(QByteArray::number(i));
}

WbDataStream &WbDataStream::operator<<(qint16 i) {
  return (WbDataStream &)append(QByteArray::number(i));
}

WbDataStream &WbDataStream::operator<<(qint32 i) {
  return (WbDataStream &)append(QByteArray::number(i));
}

WbDataStream &WbDataStream::operator<<(qint64 i) {
  return (WbDataStream &)append(QByteArray::number(i));
}

WbDataStream &WbDataStream::operator<<(bool i) {
  return (WbDataStream &)append(QByteArray::number(i));
}

WbDataStream &WbDataStream::operator<<(float f) {
  return (WbDataStream &)append(QByteArray::number(f));
}

WbDataStream &WbDataStream::operator<<(double f) {
  return (WbDataStream &)append(QByteArray::number(f));
}

WbDataStream &WbDataStream::operator<<(qfloat16 f) {
  return (WbDataStream &)append(f);
}

WbDataStream &WbDataStream::operator<<(const char *s) {
  return (WbDataStream &)append(s);
}