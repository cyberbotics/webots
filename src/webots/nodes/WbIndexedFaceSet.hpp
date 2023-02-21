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

#ifndef WB_INDEXED_FACE_SET_HPP
#define WB_INDEXED_FACE_SET_HPP

#include "WbTriangleMeshGeometry.hpp"

class WbCoordinate;
class WbNormal;
class WbTextureCoordinate;
class WbVector3;

class WbIndexedFaceSet : public WbTriangleMeshGeometry {
  Q_OBJECT

public:
  // constructors and destructor
  explicit WbIndexedFaceSet(WbTokenizer *tokenizer = NULL);
  WbIndexedFaceSet(const WbIndexedFaceSet &other);
  explicit WbIndexedFaceSet(const WbNode &other);
  virtual ~WbIndexedFaceSet();

  // reimplemented public functions
  int nodeType() const override { return WB_NODE_INDEXED_FACE_SET; }
  void preFinalize() override;
  void postFinalize() override;
  void createResizeManipulator() override;
  void attachResizeManipulator() override;
  void reset(const QString &id) override;

  // field accessors
  WbCoordinate *coord() const;
  WbNormal *normal() const;
  WbTextureCoordinate *texCoord() const;
  const WbMFInt *coordIndex() const { return static_cast<const WbMFInt *>(mCoordIndex); }
  const WbMFInt *normalIndex() const { return static_cast<const WbMFInt *>(mNormalIndex); }
  const WbMFInt *texCoordIndex() const { return static_cast<const WbMFInt *>(mTexCoordIndex); }
  const WbSFDouble *creaseAngle() const { return static_cast<const WbSFDouble *>(mCreaseAngle); }
  const WbSFBool *ccw() const { return static_cast<const WbSFBool *>(mCcw); }
  const WbSFBool *normalPerVertex() const { return static_cast<const WbSFBool *>(mNormalPerVertex); }

  // Rescaling and translating
  void rescale(const WbVector3 &v) override;
  void rescaleAndTranslate(int coordinate, double scale, double translation);
  void rescaleAndTranslate(double factor, const WbVector3 &t);
  void rescaleAndTranslate(const WbVector3 &scale, const WbVector3 &translation);
  void translate(const WbVector3 &v);

  void updateTriangleMesh(bool issueWarnings = true) override;

  uint64_t computeHash() const override;

  QStringList fieldsToSynchronizeWithX3D() const override;

signals:
  void validIndexedFaceSetInserted();

protected:
  bool areSizeFieldsVisibleAndNotRegenerator() const override;
  bool exportNodeHeader(WbWriter &writer) const override;

private:
  WbIndexedFaceSet &operator=(const WbIndexedFaceSet &);  // non copyable
  WbNode *clone() const override { return new WbIndexedFaceSet(*this); }
  void init();

  // user accessible fields
  WbSFNode *mCoord;
  WbSFNode *mNormal;
  WbSFNode *mTexCoord;
  WbSFBool *mCcw;
  WbSFBool *mNormalPerVertex;
  WbMFInt *mCoordIndex;
  WbMFInt *mNormalIndex;
  WbMFInt *mTexCoordIndex;
  WbSFDouble *mCreaseAngle;

private slots:
  void updateCoord();
  void updateNormal();
  void updateTexCoord();
  void updateCcw();
  void updateNormalPerVertex();
  void updateCoordIndex();
  void updateNormalIndex();
  void updateTexCoordIndex();
  void updateCreaseAngle();
};

#endif
