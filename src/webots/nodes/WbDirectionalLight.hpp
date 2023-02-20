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

#ifndef WB_DIRECTIONAL_LIGHT_HPP
#define WB_DIRECTIONAL_LIGHT_HPP

#include "WbLight.hpp"

class WbVector3;

struct WrDirectionalLight;

class WbDirectionalLight : public WbLight {
  Q_OBJECT

public:
  // constructors and destructor
  explicit WbDirectionalLight(WbTokenizer *tokenizer = NULL);
  WbDirectionalLight(const WbDirectionalLight &other);
  explicit WbDirectionalLight(const WbNode &other);
  virtual ~WbDirectionalLight();

  // reimplemented public functions
  int nodeType() const override { return WB_NODE_DIRECTIONAL_LIGHT; }
  void createWrenObjects() override;
  void preFinalize() override;
  void postFinalize() override;

  // specific functions
  const WbVector3 &direction() const;

  QStringList fieldsToSynchronizeWithX3D() const override;

private slots:
  void updateDirection();

private:
  // user accessible fields
  WbSFVector3 *mDirection;

  WrDirectionalLight *mWrenLight;

  WbDirectionalLight &operator=(const WbDirectionalLight &);  // non copyable
  WbNode *clone() const override { return new WbDirectionalLight(*this); }
  void init();
  void applyLightIntensityToWren() override;
  void applyLightColorToWren() override;
  void applyLightVisibilityToWren() override;
  void applyLightShadowsToWren() override;
  void applyLightDirectionToWren();
};

#endif
