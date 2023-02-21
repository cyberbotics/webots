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

#ifndef WB_VRML_WRITER_HPP
#define WB_VRML_WRITER_HPP

//
// Description: a text stream specialized for writing indented VRML or X3D
//

#include <QtCore/QHash>
#include <QtCore/QMap>

#include "WbVector3.hpp"

class QIODevice;

class WbNode;
class WbVector2;
class WbVector4;
class WbRotation;
class WbQuaternion;
class WbRgb;

class WbWriter {
public:
  WbWriter(QIODevice *device, const QString &fileName);
  WbWriter(QString *target, const QString &fileName);
  virtual ~WbWriter();

  bool isX3d() const { return mType == X3D; }
  bool isProto() const { return mType == PROTO; }
  bool isUrdf() const { return mType == URDF; }
  bool isWebots() const { return mType == VRML_SIM || mType == PROTO; }
  bool isWritingToFile() const { return mIsWritingToFile; }
  QString *string() const { return mString; };
  QString path() const;

  void writeLiteralString(const QString &string);
  void writeMFStart();
  void writeMFSeparator(bool first, bool smallSeparator);
  void writeMFEnd(bool empty);
  void writeFieldStart(const QString &name, bool x3dQuote);
  void writeFieldEnd(bool x3dQuote);

  WbVector3 jointOffset() const { return mJointOffset; }
  void setJointOffset(const WbVector3 &offset) { mJointOffset = offset; }

  // change current indentation
  void increaseIndent() { mIndent++; }
  void decreaseIndent() { mIndent--; }

  // write current indentation
  void indent();

  // write .wbt, .x3d or .urdf header and footer based on VrmlType
  void writeHeader(const QString &title);
  void writeFooter(const QStringList *info = NULL);

  void setRootNode(WbNode *node) { mRootNode = node; }
  WbNode *rootNode() const { return mRootNode; }
  void trackDeclaration(const QString &protoName, const QString &protoUrl) {
    mTrackedDeclarations.append(std::pair<QString, QString>(protoName, protoUrl));
  };
  QList<std::pair<QString, QString>> declarations() const { return mTrackedDeclarations; };

  QMap<uint64_t, QString> &indexedFaceSetDefMap() { return mIndexedFaceSetDefMap; }
  WbWriter &operator<<(const QString &s);
  WbWriter &operator<<(char);
  WbWriter &operator<<(int);
  WbWriter &operator<<(unsigned int);
  WbWriter &operator<<(float);
  WbWriter &operator<<(double);
  WbWriter &operator<<(const WbVector2 &v);
  WbWriter &operator<<(const WbVector3 &v);
  WbWriter &operator<<(const WbVector4 &v);
  WbWriter &operator<<(const WbRotation &r);
  WbWriter &operator<<(const WbQuaternion &q);
  WbWriter &operator<<(const WbRgb &rgb);

  static QString relativeTexturesPath() { return "textures/"; }
  static QString relativeMeshesPath() { return "meshes/"; }

private:
  void setType();

  enum Type { VRML_SIM, X3D, PROTO, URDF };
  QString *mString;
  QIODevice *mDevice;
  QString mFileName;
  Type mType;
  int mIndent;
  QMap<uint64_t, QString> mIndexedFaceSetDefMap;
  bool mIsWritingToFile;
  WbVector3 mJointOffset;
  // variables used by 'convert root to basenode' writer
  WbNode *mRootNode;
  QList<std::pair<QString, QString>> mTrackedDeclarations;  // keep track of declarations that need to change level
};

#endif
