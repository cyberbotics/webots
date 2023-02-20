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

#include "WbWorldInfo.hpp"

#include "WbApplicationInfo.hpp"
#include "WbContactProperties.hpp"
#include "WbDamping.hpp"
#include "WbField.hpp"
#include "WbFieldChecker.hpp"
#include "WbGroup.hpp"
#include "WbMFNode.hpp"
#include "WbMFString.hpp"
#include "WbMathsUtilities.hpp"
#include "WbOdeContext.hpp"
#include "WbParser.hpp"
#include "WbPreferences.hpp"
#include "WbProtoTemplateEngine.hpp"
#include "WbReceiver.hpp"
#include "WbSFNode.hpp"
#include "WbSFVector3.hpp"
#include "WbTokenizer.hpp"
#include "WbWorld.hpp"
#include "WbWrenRenderingContext.hpp"

#include <QtCore/QRegularExpression>

void WbWorldInfo::init(const WbVersion *version) {
  mInfo = findMFString("info");
  mTitle = findSFString("title");
  mWindow = findSFString("window");
  mCfm = findSFDouble("CFM");
  mErp = findSFDouble("ERP");
  mPhysics = findSFString("physics");
  mBasicTimeStep = findSFDouble("basicTimeStep");
  mFps = findSFDouble("FPS");
  mOptimalThreadCount = findSFInt("optimalThreadCount");
  mPhysicsDisableTime = findSFDouble("physicsDisableTime");
  mPhysicsDisableLinearThreshold = findSFDouble("physicsDisableLinearThreshold");
  mPhysicsDisableAngularThreshold = findSFDouble("physicsDisableAngularThreshold");
  mDefaultDamping = findSFNode("defaultDamping");
  mInkEvaporation = findSFDouble("inkEvaporation");
  mGravity = findSFDouble("gravity");
  mCoordinateSystem = findSFString("coordinateSystem");
  WbField *northDirectionField = findField("northDirection");
  const WbSFVector3 *const northDirection = findSFVector3("northDirection");
  if (version && *version < WbVersion(2020, 1, 0, true)) {
    mGravity->setValue(WbParser::legacyGravity());
    mCoordinateSystem->setValue("NUE");  // default value for Webots < R2020b
    if (northDirection->value() == WbVector3(1.0, 0.0, 0.0))
      northDirectionField->reset();
    else if (northDirection->value() == WbVector3(0.0, 0.0, 1.0)) {
      northDirectionField->reset();
      mCoordinateSystem->setValue("EUN");
    } else if (!northDirectionField->isDefault())
      parsingWarn(tr("The 'northDirection' field is deprecated, according to the 'coordinateSystem' field, the north is "
                     "aligned along the x-axis."));
  } else if (northDirection->value() == WbVector3(0.0, 0.0, 1.0) && mCoordinateSystem->value() == "NUE") {
    northDirectionField->reset();
    mCoordinateSystem->setValue("EUN");
  } else if (!northDirectionField->isDefault())
    parsingWarn(tr("The 'northDirection' field is deprecated, please use the 'coordinateSystem' field instead."));

  WbProtoTemplateEngine::setCoordinateSystem(mCoordinateSystem->value());
  mGpsCoordinateSystem = findSFString("gpsCoordinateSystem");
  mGpsReference = findSFVector3("gpsReference");
  mLineScale = findSFDouble("lineScale");
  mDragForceScale = findSFDouble("dragForceScale");
  mDragTorqueScale = findSFDouble("dragTorqueScale");
  mRandomSeed = findSFInt("randomSeed");
  mContactProperties = findMFNode("contactProperties");

  mPhysicsReceiver = NULL;

  if (findSFString("fast2d")->value() != "")
    parsingWarn(tr("fast2d plugin are not supported anymore, if you don't want to simulate dynamic, you can use the built-in "
                   "kinematic mode of Webots."));
}

WbWorldInfo::WbWorldInfo(WbTokenizer *tokenizer) : WbBaseNode("WorldInfo", tokenizer) {
  init(tokenizer ? &tokenizer->fileVersion() : &WbApplicationInfo::version());
}

WbWorldInfo::WbWorldInfo(const WbWorldInfo &other) : WbBaseNode(other) {
  init();
}

WbWorldInfo::WbWorldInfo(const WbNode &other) : WbBaseNode(other) {
  init();
}

WbWorldInfo::~WbWorldInfo() {
  delete mPhysicsReceiver;
}

void WbWorldInfo::downloadAssets() {
  const int size = mContactProperties->size();
  for (int i = 0; i < size; ++i) {
    WbContactProperties *const cp = static_cast<WbContactProperties *>(mContactProperties->item(i));
    cp->downloadAssets();
  }
}

void WbWorldInfo::preFinalize() {
  WbBaseNode::preFinalize();

  if (defaultDamping())
    defaultDamping()->preFinalize();

  if (!mPhysics->value().isEmpty() || mPhysics->value() != "<none>")
    mPhysicsReceiver = WbReceiver::createPhysicsReceiver();

  updateGravity();
  updateCfm();
  updateErp();
  updateBasicTimeStep();
  updateFps();
  updateLineScale();
  updateDragForceScale();
  updateDragTorqueScale();
  updateRandomSeed();
  updateDefaultDamping();
  updateGpsCoordinateSystem();
  WbProtoTemplateEngine::setCoordinateSystem(mCoordinateSystem->value());

  const int size = mContactProperties->size();
  for (int i = 0; i < size; ++i) {
    WbContactProperties *const cp = static_cast<WbContactProperties *>(mContactProperties->item(i));
    cp->preFinalize();
  }
}

void WbWorldInfo::postFinalize() {
  WbBaseNode::postFinalize();

  if (defaultDamping())
    defaultDamping()->postFinalize();

  connect(mTitle, &WbSFString::changed, this, &WbWorldInfo::titleChanged);
  connect(mGravity, &WbSFDouble::changed, this, &WbWorldInfo::updateGravity);
  connect(mCfm, &WbSFDouble::changed, this, &WbWorldInfo::updateCfm);
  connect(mErp, &WbSFDouble::changed, this, &WbWorldInfo::updateErp);
  connect(mBasicTimeStep, &WbSFDouble::changed, this, &WbWorldInfo::updateBasicTimeStep);
  connect(mOptimalThreadCount, &WbSFInt::changed, this, &WbWorldInfo::updateOptimalThreadCount);
  connect(mOptimalThreadCount, &WbSFInt::changed, this, &WbWorldInfo::displayOptimalThreadCountWarning);
  connect(mFps, &WbSFDouble::changed, this, &WbWorldInfo::updateFps);
  connect(mLineScale, &WbSFDouble::changed, this, &WbWorldInfo::updateLineScale);
  connect(mDragForceScale, &WbSFDouble::changed, this, &WbWorldInfo::updateDragForceScale);
  connect(mDragTorqueScale, &WbSFDouble::changed, this, &WbWorldInfo::updateDragTorqueScale);
  connect(mRandomSeed, &WbSFInt::changed, this, &WbWorldInfo::updateRandomSeed);
  connect(mPhysicsDisableTime, &WbSFDouble::changed, this, &WbWorldInfo::physicsDisableChanged);
  connect(mPhysicsDisableLinearThreshold, &WbSFDouble::changed, this, &WbWorldInfo::physicsDisableChanged);
  connect(mPhysicsDisableAngularThreshold, &WbSFDouble::changed, this, &WbWorldInfo::physicsDisableChanged);
  connect(mDefaultDamping, &WbSFNode::changed, this, &WbWorldInfo::updateDefaultDamping);
  connect(mCoordinateSystem, &WbSFString::changed, this, &WbWorldInfo::updateCoordinateSystem);
  connect(mCoordinateSystem, &WbSFString::changed, this, &WbWorldInfo::updateGravity);

  connect(mGpsCoordinateSystem, &WbSFString::changed, this, &WbWorldInfo::updateGpsCoordinateSystem);
  connect(mGpsReference, &WbSFString::changed, this, &WbWorldInfo::gpsReferenceChanged);

  const int size = mContactProperties->size();
  for (int i = 0; i < size; ++i) {
    WbContactProperties *const cp = static_cast<WbContactProperties *>(mContactProperties->item(i));
    cp->postFinalize();
    connect(cp, &WbContactProperties::valuesChanged, this, &WbWorldInfo::updateContactProperties);
  }

  connect(mContactProperties, &WbMFNode::changed, this, &WbWorldInfo::updateContactProperties);

  WbWorld::instance()->setWorldInfo(this);
}

void WbWorldInfo::reset(const QString &id) {
  WbBaseNode::reset(id);

  for (int i = 0; i < mContactProperties->size(); ++i)
    mContactProperties->item(i)->reset(id);
  WbNode *const d = mDefaultDamping->value();
  if (d)
    d->reset(id);
}

double WbWorldInfo::lineScale() const {
  return mLineScale->value();
}

int WbWorldInfo::contactPropertiesCount() const {
  return mContactProperties->size();
}

WbContactProperties *WbWorldInfo::contactProperties(int index) const {
  return static_cast<WbContactProperties *>(mContactProperties->item(index));
}

WbDamping *WbWorldInfo::defaultDamping() const {
  return static_cast<WbDamping *>(mDefaultDamping->value());
}

void WbWorldInfo::createWrenObjects() {
  WbBaseNode::createWrenObjects();
  WbWrenRenderingContext::instance()->setLineScale(static_cast<float>(lineScale()));
}

void WbWorldInfo::createOdeObjects() {
  WbBaseNode::createOdeObjects();

  applyToOdeGravity();
  applyToOdeCfm();
  applyToOdeErp();
  applyToOdeGlobalDamping();
  applyToOdePhysicsDisableTime();
}

void WbWorldInfo::updateBasicTimeStep() {
  WbFieldChecker::resetDoubleIfNonPositive(this, mBasicTimeStep, 32.0);
}

void WbWorldInfo::updateFps() {
  WbFieldChecker::resetDoubleIfNonPositive(this, mFps, 60.0);
}

void WbWorldInfo::displayOptimalThreadCountWarning() {
  int threadPreferenceNumber = WbPreferences::instance()->value("General/numberOfThreads", 1).toInt();
  if (mOptimalThreadCount->value() > 1 and threadPreferenceNumber > 1)
    parsingWarn(
      tr("Physics multi-threading is enabled. "
         "This can have a noticeable impact on the simulation speed (negative or positive depending on the simulated world). "
         "In case of multi-threading, simulation replicability is not guaranteed. "));
}

void WbWorldInfo::updateOptimalThreadCount() {
  // this function is called only on field update,
  // loading a world where the 'optimalThreadCount' field is higher than the limit set in the preferences will therefore not
  // raise any warning
  int threadPreferenceNumber = WbPreferences::instance()->value("General/numberOfThreads", 1).toInt();
  if (mOptimalThreadCount->value() > threadPreferenceNumber)
    parsingWarn(tr("A limit of '%1' threads is set in the preferences.").arg(threadPreferenceNumber));
  else if (!WbFieldChecker::resetIntIfNonPositive(this, mOptimalThreadCount, 1))
    emit optimalThreadCountChanged();
}

void WbWorldInfo::updateLineScale() {
  if (WbFieldChecker::resetDoubleIfNegative(this, mLineScale, 0.0))
    return;

  if (areWrenObjectsInitialized())
    applyLineScaleToWren();
}

void WbWorldInfo::applyLineScaleToWren() {
  WbWrenRenderingContext::instance()->setLineScale(static_cast<float>(mLineScale->value()));
}

void WbWorldInfo::updateDragForceScale() {
  WbFieldChecker::resetDoubleIfNonPositive(this, mDragForceScale, 30.0);
}

void WbWorldInfo::updateDragTorqueScale() {
  WbFieldChecker::resetDoubleIfNonPositive(this, mDragTorqueScale, 5.0);
}

void WbWorldInfo::updateRandomSeed() {
  WbFieldChecker::resetIntIfNegativeAndNotDisabled(this, mRandomSeed, 0, -1);
  emit randomSeedChanged();
}

void WbWorldInfo::applyToOdePhysicsDisableTime() {
  WbOdeContext::instance()->setPhysicsDisableTime(mPhysicsDisableTime->value());
}

void WbWorldInfo::updateGravity() {
  updateGravityBasis();
  if (areOdeObjectsCreated())
    applyToOdeGravity();
}

void WbWorldInfo::applyToOdeGravity() {
  WbOdeContext::instance()->setGravity(mGravityVector.x(), mGravityVector.y(), mGravityVector.z());
  emit globalPhysicsPropertiesChanged();
}

void WbWorldInfo::updateCfm() {
  if (WbFieldChecker::resetDoubleIfNonPositive(this, mCfm, 0.00001))
    return;

  if (areOdeObjectsCreated())
    applyToOdeCfm();
}

void WbWorldInfo::applyToOdeCfm() {
  WbOdeContext::instance()->setCfm(mCfm->value());
  emit globalPhysicsPropertiesChanged();
}

void WbWorldInfo::updateErp() {
  if (WbFieldChecker::resetDoubleIfNotInRangeWithIncludedBounds(this, mErp, 0.0, 1.0, 0.2))
    return;

  if (areOdeObjectsCreated())
    applyToOdeErp();
}

void WbWorldInfo::applyToOdeErp() {
  WbOdeContext::instance()->setErp(mErp->value());
  emit globalPhysicsPropertiesChanged();
}

void WbWorldInfo::updateDefaultDamping() {
  if (areOdeObjectsCreated())
    applyToOdeGlobalDamping();
}

void WbWorldInfo::applyToOdeGlobalDamping() {
  const WbDamping *const damping = defaultDamping();
  if (damping) {
    connect(damping, &WbDamping::changed, this, &WbWorldInfo::updateDefaultDamping, Qt::UniqueConnection);

    // convert damping per second (specified in the Scene Tree) in damping per step (for ODE)
    const double ts = basicTimeStep() * 0.001;
    const double linear = 1.0 - pow(1.0 - damping->linear(), ts);
    const double angular = 1.0 - pow(1.0 - damping->angular(), ts);
    WbOdeContext::instance()->setDamping(linear, angular);
  } else {
    // default ODE damping is 0.0
    WbOdeContext::instance()->setDamping(0.0, 0.0);
  }

  emit globalPhysicsPropertiesChanged();
}

// Computes an orthonormal basis whose 'yaw unit vector' is the opposite of the normalized gravity vector
void WbWorldInfo::updateGravityBasis() {
  const QString &system = mCoordinateSystem->value();
  assert(system.size() == 3);
  mNorthVector = WbVector3(system[0] == 'N' ? 1 : 0, system[1] == 'N' ? 1 : 0, system[2] == 'N' ? 1 : 0);
  mEastVector = WbVector3(system[0] == 'E' ? 1 : 0, system[1] == 'E' ? 1 : 0, system[2] == 'E' ? 1 : 0);
  mUpVector = WbVector3(system[0] == 'U' ? 1 : 0, system[1] == 'U' ? 1 : 0, system[2] == 'U' ? 1 : 0);
  mGravityUnitVector = -mUpVector;
  mGravityVector = mGravityUnitVector * mGravity->value();
}

void WbWorldInfo::updateCoordinateSystem() {
  warn(tr("Please save and revert the world so that the change of coordinate system is taken into account when reloading "
          "procedural PROTO nodes."));
}

void WbWorldInfo::updateGpsCoordinateSystem() {
  if (mGpsCoordinateSystem->value() != "local" && mGpsCoordinateSystem->value() != "WGS84") {
    mGpsCoordinateSystem->setValue("local");
    parsingWarn(tr("'gpsCoordinateSystem' must either be 'local' or 'WGS84'. Reset to default value 'local'."));
  }
  emit gpsCoordinateSystemChanged();
}

void WbWorldInfo::updateContactProperties() {
  if (areOdeObjectsCreated())
    emit globalPhysicsPropertiesChanged();
}

// e.g. '"Aldebaran's >"' to '"Aldebaran&#39;s &gt;"'
static QString forgeHtmlEscapedString(const QString &s) {
  QString r = s;
  r = r.replace(QRegularExpression("^\""), "").replace(QRegularExpression("\"$"), "");  // remove first and last double quotes
  r = r.toHtmlEscaped().replace("'", "&#39;");  // replace the problematic HTML characters by their codes
  return QString("\"%1\"").arg(r);              // restore the suffix and prefix double quotes
}

void WbWorldInfo::exportNodeFields(WbWriter &writer) const {
  if (writer.isX3d()) {
    QString titleString = forgeHtmlEscapedString(mTitle->toString());
    if (titleString.size() > 2)  // at least 2 double quotes
      writer << " title=" << titleString;

    if (mInfo->size() > 0) {
      writer << " info='";
      for (int i = 0; i < mInfo->size(); ++i) {
        QString infoString = forgeHtmlEscapedString(mInfo->itemToString(i));
        writer << infoString;
        if (i != mInfo->size() - 1)
          writer << " ";
      }
      writer << "'";
    }

    writer << " basicTimeStep=\'" << mBasicTimeStep->value() << "\'";
    writer << " coordinateSystem=\'" << mCoordinateSystem->value() << "\'";
    writer << " lineScale=\'" << mLineScale->value() << "\'";
  } else
    WbBaseNode::exportNodeFields(writer);
}
