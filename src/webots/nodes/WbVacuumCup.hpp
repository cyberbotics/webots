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

#ifndef WB_VACUUM_CUP_HPP
#define WB_VACUUM_CUP_HPP

#include <WbSolidDevice.hpp>

class WbSensor;

class WbVacuumCup : public WbSolidDevice {
  Q_OBJECT

public:
  // constructors and destructor
  explicit WbVacuumCup(WbTokenizer *tokenizer = NULL);
  WbVacuumCup(const WbVacuumCup &other);
  explicit WbVacuumCup(const WbNode &other);
  virtual ~WbVacuumCup();

  // reimplemented public functions
  int nodeType() const override { return WB_NODE_VACUUM_CUP; }
  void preFinalize() override;
  void postFinalize() override;
  void handleMessage(QDataStream &stream) override;
  void writeAnswer(WbDataStream &stream) override;
  void writeConfigure(WbDataStream &) override;
  void prePhysicsStep(double ms) override;
  void postPhysicsStep() override;
  bool refreshSensorIfNeeded() override;
  void reset(const QString &id) override;
  void save(const QString &id) override;

  bool isWaitingForConnection();
  void addCollidedSolid(WbSolid *solid, const double depth);

private:
  // fields
  WbSFBool *mIsOn;               // current state
  WbSFDouble *mTensileStrength;  // max pull force that the connector can withstand without breaking (Newtons)
  WbSFDouble *mShearStrength;    // max shear force that the connector can withstand without breaking (Newtons)

  // other stuff
  dJointID mFixedJoint;  // ODE joint that does the connection
  WbSolid *mSolid;       // connected object or NULL
  WbSensor *mSensor;     // presence sensor
  bool mValue;
  bool mNeedToReconfigure;
  QMap<QString, bool> mIsInitiallyOn;
  QList<std::pair<WbSolid *, const double>> mCollidedSolidList;  // list of Solid that collided with the deepest contact depth

  WbVacuumCup &operator=(const WbVacuumCup &);  // non copyable
  WbNode *clone() const override { return new WbVacuumCup(*this); }
  void addConfigure(WbDataStream &);

  void attachToSolid();
  void detachFromSolid();
  void createFixedJoint(WbSolid *other);
  void destroyFixedJoint();
  void turnOn();
  void turnOff();
  void computeValue();
  void detachIfForceExceedStrength();
  double getEffectiveTensileStrength() const;
  double getEffectiveShearStrength() const;
  void init();

private slots:
  void updateIsOn();
  void updateTensileStrength();
  void updateShearStrength();
};

#endif
