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

#include "WbWriter.hpp"

#include "WbApplicationInfo.hpp"
#include "WbQuaternion.hpp"
#include "WbRgb.hpp"
#include "WbRotation.hpp"
#include "WbVector2.hpp"
#include "WbVector4.hpp"
#include "WbVersion.hpp"

#include <QtCore/QFileInfo>

WbWriter::WbWriter(QIODevice *device, const QString &fileName) :
  mString(NULL),
  mDevice(device),
  mFileName(fileName),
  mIndent(0),
  mIsWritingToFile(true),
  mJointOffset(0.0, 0.0, 0.0),
  mRootNode(NULL) {
  setType();
}

WbWriter::WbWriter(QString *target, const QString &fileName) :
  mString(target),
  mDevice(NULL),
  mFileName(fileName),
  mIndent(0),
  mIsWritingToFile(false),
  mJointOffset(0.0, 0.0, 0.0),
  mRootNode(NULL) {
  setType();
}

WbWriter::~WbWriter() {
}

void WbWriter::setType() {
  if (mFileName.endsWith(".wbt", Qt::CaseInsensitive))
    mType = VRML_SIM;
  else if (mFileName.endsWith(".x3d", Qt::CaseInsensitive))
    mType = X3D;
  else if (mFileName.endsWith(".proto", Qt::CaseInsensitive))
    mType = PROTO;
  else if (mFileName.endsWith(".urdf", Qt::CaseInsensitive))
    mType = URDF;
}

QString WbWriter::path() const {
  QFileInfo p(mFileName);
  return p.path();
}

void WbWriter::writeMFStart() {
  if (!isX3d() && !isUrdf()) {
    *this << "[";
    increaseIndent();
  }
}

void WbWriter::writeMFSeparator(bool first, bool smallSeparator) {
  if (!isX3d() && !isUrdf()) {
    if (smallSeparator && !first)
      *this << ", ";
    else {
      *this << "\n";
      indent();
    }
  } else if (!first && !isUrdf())  // X3D
    *this << " ";
}

void WbWriter::writeMFEnd(bool empty) {
  if (!isX3d() && !isUrdf()) {
    decreaseIndent();
    if (!empty) {
      *this << "\n";
      indent();
    }
    *this << "]";
  }
}

void WbWriter::writeFieldStart(const QString &name, bool x3dQuote) {
  if (isX3d()) {
    *this << name + "=";
    if (x3dQuote)
      *this << "\'";
  } else {
    indent();
    *this << name + " ";
  }
}

void WbWriter::writeFieldEnd(bool x3dQuote) {
  if (isX3d()) {
    if (x3dQuote)
      *this << "\'";
  } else
    *this << "\n";
}

void WbWriter::writeLiteralString(const QString &string) {
  QString text(string);
  if (isX3d()) {
    text.replace("&", "&amp;");
    text.replace("<", "&lt;");
    text.replace(">", "&gt;");
  }
  text.replace("\\", "\\\\");   // replace '\' by '\\'
  text.replace("\"", "\\\"");   // replace '"' by '\"'
  *this << '"' << text << '"';  // add double quotes
}

void WbWriter::indent() {
  for (int i = 0; i < mIndent; ++i)
    *this << "  ";
}

void WbWriter::writeHeader(const QString &title) {
  switch (mType) {
    case VRML_SIM:
      *this << QString("#VRML_SIM %1 utf8\n").arg(WbApplicationInfo::version().toString(false));
      return;
    case X3D:
      *this << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
      *this << "<!DOCTYPE X3D PUBLIC \"ISO//Web3D//DTD X3D 3.0//EN\" \"http://www.web3d.org/specifications/x3d-3.0.dtd\">\n";
      *this << "<X3D version=\"3.0\" profile=\"Immersive\" xmlns:xsd=\"http://www.w3.org/2001/XMLSchema-instance\" "
               "xsd:noNamespaceSchemaLocation=\"http://www.web3d.org/specifications/x3d-3.0.xsd\">\n";
      *this << "<head>\n";
      *this << "<meta name=\"generator\" content=\"Webots\" />\n";
      *this << "<meta name=\"version\" content=\"" + WbApplicationInfo::version().toString(false) + "\" />\n";
      *this << "</head>\n";
      *this << "<Scene>\n";
      return;
    case URDF:
      *this << "<?xml version=\"1.0\"?>\n";
      *this << "<robot name=\"" + title + "\" xmlns:xacro=\"http://ros.org/wiki/xacro\">\n";
      return;
    default:
      return;
  }
}

void WbWriter::writeFooter(const QStringList *info) {
  if (isX3d()) {
    *this << "</Scene>\n";
    *this << "</X3D>\n";
  } else if (isUrdf())
    *this << "</robot>\n";
}

WbWriter &WbWriter::operator<<(const QString &s) {
  if (mString)
    *mString += s;
  else
    mDevice->write(s.toUtf8());
  return *this;
}

WbWriter &WbWriter::operator<<(char c) {
  *this << QString(c);
  return *this;
}

WbWriter &WbWriter::operator<<(int i) {
  *this << QString::number(i);
  return *this;
}

WbWriter &WbWriter::operator<<(unsigned int i) {
  *this << QString::number(i);
  return *this;
}

WbWriter &WbWriter::operator<<(float f) {
  *this << WbPrecision::doubleToString(f, WbPrecision::FLOAT_MAX);
  return *this;
}

WbWriter &WbWriter::operator<<(double f) {
  *this << WbPrecision::doubleToString(f, WbPrecision::DOUBLE_MAX);
  return *this;
}

WbWriter &WbWriter::operator<<(const WbVector2 &v) {
  *this << v.toString(WbPrecision::DOUBLE_MAX);
  return *this;
}

WbWriter &WbWriter::operator<<(const WbVector3 &v) {
  *this << v.toString(WbPrecision::DOUBLE_MAX);
  return *this;
}

WbWriter &WbWriter::operator<<(const WbVector4 &v) {
  *this << v.toString(WbPrecision::DOUBLE_MAX);
  return *this;
}

WbWriter &WbWriter::operator<<(const WbRotation &r) {
  *this << r.toString(WbPrecision::DOUBLE_MAX);
  return *this;
}

WbWriter &WbWriter::operator<<(const WbQuaternion &q) {
  *this << q.toString(WbPrecision::DOUBLE_MAX);
  return *this;
}

WbWriter &WbWriter::operator<<(const WbRgb &rgb) {
  *this << rgb.toString(WbPrecision::FLOAT_MAX);
  return *this;
}
