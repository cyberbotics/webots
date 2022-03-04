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

#include "WbVrmlWriter.hpp"

#include "WbApplicationInfo.hpp"
#include "WbQuaternion.hpp"
#include "WbRgb.hpp"
#include "WbRotation.hpp"
#include "WbVector2.hpp"
#include "WbVector4.hpp"
#include "WbVersion.hpp"

#include <QtCore/QFileInfo>

WbVrmlWriter::WbVrmlWriter(QIODevice *device, const QString &fileName) :
  mString(NULL),
  mDevice(device),
  mFileName(fileName),
  mIndent(0),
  mRootNode(NULL),
  mIsWritingToFile(true),
  mJointOffset(0.0, 0.0, 0.0) {
  setVrmlType();
}

WbVrmlWriter::WbVrmlWriter(QString *target, const QString &fileName) :
  mString(target),
  mDevice(NULL),
  mFileName(fileName),
  mIndent(0),
  mRootNode(NULL),
  mIsWritingToFile(false),
  mJointOffset(0.0, 0.0, 0.0) {
  setVrmlType();
}

WbVrmlWriter::~WbVrmlWriter() {
}

void WbVrmlWriter::setVrmlType() {
  if (mFileName.endsWith(".wbt", Qt::CaseInsensitive))
    mVrmlType = VRML_SIM;
  else if (mFileName.endsWith(".wbo", Qt::CaseInsensitive))
    mVrmlType = VRML_OBJ;
  else if (mFileName.endsWith(".wrl", Qt::CaseInsensitive))
    mVrmlType = VRML;
  else if (mFileName.endsWith(".x3d", Qt::CaseInsensitive))
    mVrmlType = X3D;
  else if (mFileName.endsWith(".proto", Qt::CaseInsensitive))
    mVrmlType = PROTO;
  else if (mFileName.endsWith(".urdf", Qt::CaseInsensitive))
    mVrmlType = URDF;
}

QString WbVrmlWriter::path() const {
  QFileInfo p(mFileName);
  return p.path();
}

void WbVrmlWriter::writeMFStart() {
  if (!isX3d() && !isUrdf()) {
    *this << "[";
    increaseIndent();
  }
}

void WbVrmlWriter::writeMFSeparator(bool first, bool smallSeparator) {
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

void WbVrmlWriter::writeMFEnd(bool empty) {
  if (!isX3d() && !isUrdf()) {
    decreaseIndent();
    if (!empty) {
      *this << "\n";
      indent();
    }
    *this << "]";
  }
}

void WbVrmlWriter::writeFieldStart(const QString &name, bool x3dQuote) {
  if (isX3d()) {
    *this << name + "=";
    if (x3dQuote)
      *this << "\'";
  } else {
    indent();
    *this << name + " ";
  }
}

void WbVrmlWriter::writeFieldEnd(bool x3dQuote) {
  if (isX3d()) {
    if (x3dQuote)
      *this << "\'";
  } else
    *this << "\n";
}

void WbVrmlWriter::writeLiteralString(const QString &string) {
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

void WbVrmlWriter::indent() {
  for (int i = 0; i < mIndent; ++i)
    *this << "  ";
}

void WbVrmlWriter::writeHeader(const QString &title) {
  switch (mVrmlType) {
    case VRML:
      *this << "#VRML V2.0 utf8\n";
      return;
    case VRML_SIM:
      *this << QString("#VRML_SIM %1 utf8\n").arg(WbApplicationInfo::version().toString(false));
      return;
    case VRML_OBJ:
      *this << QString("#VRML_OBJ %1 utf8\n").arg(WbApplicationInfo::version().toString(false));
      return;
    case X3D:
      *this << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
      *this << "<!DOCTYPE X3D PUBLIC \"ISO//Web3D//DTD X3D 3.0//EN\" \"http://www.web3d.org/specifications/x3d-3.0.dtd\">\n";
      *this << "<X3D version=\"3.0\" profile=\"Immersive\" xmlns:xsd=\"http://www.w3.org/2001/XMLSchema-instance\" "
               "xsd:noNamespaceSchemaLocation=\"http://www.web3d.org/specifications/x3d-3.0.xsd\">\n";
      *this << "<head>\n";
      *this << "<meta name=\"generator\" content=\"Webots\" />\n";
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

void WbVrmlWriter::writeFooter(const QStringList *info) {
  if (isX3d()) {
    *this << "</Scene>\n";
    *this << "</X3D>\n";
  } else if (isUrdf())
    *this << "</robot>\n";
}

WbVrmlWriter &WbVrmlWriter::operator<<(const QString &s) {
  if (mString)
    *mString += s;
  else
    mDevice->write(s.toUtf8());
  return *this;
}

WbVrmlWriter &WbVrmlWriter::operator<<(char c) {
  *this << QString(c);
  return *this;
}

WbVrmlWriter &WbVrmlWriter::operator<<(int i) {
  *this << QString::number(i);
  return *this;
}

WbVrmlWriter &WbVrmlWriter::operator<<(unsigned int i) {
  *this << QString::number(i);
  return *this;
}

WbVrmlWriter &WbVrmlWriter::operator<<(float f) {
  *this << WbPrecision::doubleToString(f, WbPrecision::FLOAT_MAX);
  return *this;
}

WbVrmlWriter &WbVrmlWriter::operator<<(double f) {
  *this << WbPrecision::doubleToString(f, WbPrecision::DOUBLE_MAX);
  return *this;
}

WbVrmlWriter &WbVrmlWriter::operator<<(const WbVector2 &v) {
  *this << v.toString(WbPrecision::DOUBLE_MAX);
  return *this;
}

WbVrmlWriter &WbVrmlWriter::operator<<(const WbVector3 &v) {
  *this << v.toString(WbPrecision::DOUBLE_MAX);
  return *this;
}

WbVrmlWriter &WbVrmlWriter::operator<<(const WbVector4 &v) {
  *this << v.toString(WbPrecision::DOUBLE_MAX);
  return *this;
}

WbVrmlWriter &WbVrmlWriter::operator<<(const WbRotation &r) {
  *this << r.toString(WbPrecision::DOUBLE_MAX);
  return *this;
}

WbVrmlWriter &WbVrmlWriter::operator<<(const WbQuaternion &q) {
  *this << q.toString(WbPrecision::DOUBLE_MAX);
  return *this;
}

WbVrmlWriter &WbVrmlWriter::operator<<(const WbRgb &rgb) {
  *this << rgb.toString(WbPrecision::FLOAT_MAX);
  return *this;
}
