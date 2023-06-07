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

#ifndef WB_FLUID_HPP
#define WB_FLUID_HPP

#include "WbMatter.hpp"

class WbFluid : public WbMatter {
  Q_OBJECT

public:
  // constructors and destructor
  explicit WbFluid(WbTokenizer *tokenizer = NULL);
  WbFluid(const WbFluid &other);
  explicit WbFluid(const WbNode &other);
  virtual ~WbFluid() {}

  // reimplemented public functions
  int nodeType() const override { return WB_NODE_FLUID; }
  void preFinalize() override;
  void postFinalize() override;
  void propagateSelection(bool selected) override;
  void createWrenObjects() override;
  void createOdeObjects() override;

  // handle artifical moves triggered by the user or a Supervisor
  void jerk(bool resetVelocities = false, bool rootJerk = true) override;

  // fields
  double density() const;
  double viscosity() const;
  const WbVector3 &streamVelocity() const { return mStreamVelocity->value(); }
  dFluidID odeFluid() const { return mOdeFluid; }

public slots:
  // recursions through solid children with bounding objects for material updates
  void propagateBoundingObjectMaterialUpdate(bool onSelection = false) override;
  void updateBoundingObject() override;

protected:
  // this constructor is reserved for derived classes only
  WbFluid(const QString &modelName, WbTokenizer *tokenizer);

  void setGeomMatter(dGeomID g, WbBaseNode *node = NULL) override;

private:
  WbFluid &operator=(const WbFluid &);  // non copyable
  void init();

  // user accessible fields
  WbSFDouble *mDensity;
  WbSFDouble *mViscosity;
  WbSFVector3 *mStreamVelocity;

  dFluidID mOdeFluid;

  // Clone
  WbNode *clone() const override { return new WbFluid(*this); }
  void handleJerk() override;

  // Bounding Object
  void attachGeomsToFluid(dGeomID g);

  void createOdeGeoms() override;

private slots:
  void createOdeGeomFromInsertedGroupItem(WbBaseNode *node) override;
  void updateStreamVelocity();
  void updateDensity();
  void updateViscosity();
};

#endif
