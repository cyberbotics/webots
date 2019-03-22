// Copyright 1996-2019 Cyberbotics Ltd.
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

#include "WbWorldInfo.hpp"

#include "WbContactProperties.hpp"
#include "WbDamping.hpp"
#include "WbField.hpp"
#include "WbFieldChecker.hpp"
#include "WbGroup.hpp"
#include "WbMFNode.hpp"
#include "WbMFString.hpp"
#include "WbMathsUtilities.hpp"
#include "WbOdeContext.hpp"
#include "WbPreferences.hpp"
#include "WbReceiver.hpp"
#include "WbSFNode.hpp"
#include "WbSFVector3.hpp"
#include "WbWorld.hpp"
#include "WbWrenRenderingContext.hpp"

void WbWorldInfo::init() {
  mInfo = findMFString("info");
  mTitle = findSFString("title");
  mWindow = findSFString("window");
  mGravity = findSFVector3("gravity");
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
  mNorthDirection = findSFVector3("northDirection");
  mGpsCoordinateSystem = findSFString("gpsCoordinateSystem");
  mGpsReference = findSFVector3("gpsReference");
  mLineScale = findSFDouble("lineScale");
  mRandomSeed = findSFInt("randomSeed");
  mContactProperties = findMFNode("contactProperties");

  mPhysicsReceiver = NULL;

  if (findSFString("fast2d")->value() != "")
    warn(tr("fast2d plugin are not supported anymore, if you don't want to simulate dynamic, you can use the built-in "
            "kinematic mode of Webots."));
}

WbWorldInfo::WbWorldInfo(WbTokenizer *tokenizer) : WbBaseNode("WorldInfo", tokenizer) {
  init();
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

void WbWorldInfo::preFinalize() {
  WbBaseNode::preFinalize();

  if (defaultDamping())
    defaultDamping()->preFinalize();

  if (!mPhysics->value().isEmpty())
    mPhysicsReceiver = WbReceiver::createPhysicsReceiver();

  updateGravity();
  updateCfm();
  updateErp();
  updateBasicTimeStep();
  updateFps();
  updateLineScale();
  updateRandomSeed();
  updateDefaultDamping();
  updateNorthDirection();
  updateGpsCoordinateSystem();

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
  connect(mGravity, &WbSFVector3::changed, this, &WbWorldInfo::updateGravity);
  connect(mCfm, &WbSFDouble::changed, this, &WbWorldInfo::updateCfm);
  connect(mErp, &WbSFDouble::changed, this, &WbWorldInfo::updateErp);
  connect(mBasicTimeStep, &WbSFDouble::changed, this, &WbWorldInfo::updateBasicTimeStep);
  connect(mOptimalThreadCount, &WbSFInt::changed, this, &WbWorldInfo::updateOptimalThreadCount);
  connect(mOptimalThreadCount, &WbSFInt::changed, this, &WbWorldInfo::displayOptimalThreadCountWarning);
  connect(mFps, &WbSFDouble::changed, this, &WbWorldInfo::updateFps);
  connect(mLineScale, &WbSFDouble::changed, this, &WbWorldInfo::updateLineScale);
  connect(mRandomSeed, &WbSFInt::changed, this, &WbWorldInfo::updateRandomSeed);
  connect(mPhysicsDisableTime, &WbSFDouble::changed, this, &WbWorldInfo::physicsDisableChanged);
  connect(mPhysicsDisableLinearThreshold, &WbSFDouble::changed, this, &WbWorldInfo::physicsDisableChanged);
  connect(mPhysicsDisableAngularThreshold, &WbSFDouble::changed, this, &WbWorldInfo::physicsDisableChanged);
  connect(mDefaultDamping, &WbSFNode::changed, this, &WbWorldInfo::updateDefaultDamping);
  connect(mNorthDirection, &WbSFVector3::changed, this, &WbWorldInfo::updateNorthDirection);
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

void WbWorldInfo::reset() {
  WbBaseNode::reset();

  for (int i = 0; i < mContactProperties->size(); ++i)
    mContactProperties->item(i)->reset();
  WbNode *const d = mDefaultDamping->value();
  if (d)
    d->reset();
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
  WbFieldChecker::checkDoubleIsPositive(this, mBasicTimeStep, 32.0);
}

void WbWorldInfo::updateFps() {
  WbFieldChecker::checkDoubleIsPositive(this, mFps, 60.0);
}

void WbWorldInfo::displayOptimalThreadCountWarning() {
  int threadPreferenceNumber = WbPreferences::instance()->value("General/numberOfThreads", 1).toInt();
  if (mOptimalThreadCount->value() > 1 and threadPreferenceNumber > 1)
    warn(
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
    warn(tr("A limit of '%1' threads is set in the preferences.").arg(threadPreferenceNumber));
  else if (!WbFieldChecker::checkIntIsPositive(this, mOptimalThreadCount, 1))
    emit optimalThreadCountChanged();
}

void WbWorldInfo::updateLineScale() {
  if (WbFieldChecker::checkDoubleIsNonNegative(this, mLineScale, 0.0))
    return;

  if (areWrenObjectsInitialized())
    applyLineScaleToWren();
}

void WbWorldInfo::applyLineScaleToWren() {
  WbWrenRenderingContext::instance()->setLineScale(static_cast<float>(mLineScale->value()));
}

void WbWorldInfo::updateRandomSeed() {
  WbFieldChecker::checkIntIsNonNegativeOrDisabled(this, mRandomSeed, 0, -1);
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
  const WbVector3 &gravity = mGravity->value();
  WbOdeContext::instance()->setGravity(gravity.x(), gravity.y(), gravity.z());
  emit globalPhysicsPropertiesChanged();
}

void WbWorldInfo::updateCfm() {
  if (WbFieldChecker::checkDoubleIsPositive(this, mCfm, 0.00001))
    return;

  if (areOdeObjectsCreated())
    applyToOdeCfm();
}

void WbWorldInfo::applyToOdeCfm() {
  WbOdeContext::instance()->setCfm(mCfm->value());
  emit globalPhysicsPropertiesChanged();
}

void WbWorldInfo::updateErp() {
  if (WbFieldChecker::checkDoubleInRangeWithIncludedBounds(this, mErp, 0.0, 1.0, 0.2))
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
  if (!gravity().isNull()) {
    mGravityUnitVector = gravity().normalized();
    WbMathsUtilities::orthoBasis(-gravity(), mGravityBasis);
  } else {
    mGravityUnitVector.setXyz(0.0, -1.0, 0.0);
    mGravityBasis[X].setXyz(1.0, 0.0, 0.0);
    mGravityBasis[Y].setXyz(0.0, 1.0, 0.0);
    mGravityBasis[Z].setXyz(0.0, 0.0, 1.0);
  }
}

void WbWorldInfo::updateNorthDirection() {
  if (mNorthDirection->value().isNull()) {
    mNorthDirection->setValue(1, 0, 0);
    warn(tr("'northDirection' must be a unit vector. Reset to default value (1, 0, 0)."));
  }
}

void WbWorldInfo::updateGpsCoordinateSystem() {
  if (mGpsCoordinateSystem->value().compare("local") != 0 and mGpsCoordinateSystem->value().compare("WGS84") != 0) {
    mGpsCoordinateSystem->setValue("local");
    warn(tr("'gpsCoordinateSystem' must either be 'local' or 'WGS84'. Reset to default value 'local'."));
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
  r = r.replace(QRegExp("^\""), "").replace(QRegExp("\"$"), "");  // remove first and last double quotes
  r = r.toHtmlEscaped().replace("'", "&#39;");                    // replace the problematic HTML characters by their codes
  return QString("\"%1\"").arg(r);                                // restore the suffix and prefix double quotes
}

void WbWorldInfo::exportNodeFields(WbVrmlWriter &writer) const {
  if (writer.isX3d()) {
    QString title = forgeHtmlEscapedString(mTitle->toString());
    if (title.size() > 2)  // at least 2 double quotes
      writer << " title=" << title;

    if (mInfo->size() > 0) {
      writer << " info='";
      for (int i = 0; i < mInfo->size(); ++i) {
        QString info = forgeHtmlEscapedString(mInfo->itemToString(i));
        writer << info;
        if (i != mInfo->size() - 1)
          writer << " ";
      }
      writer << "'";
    }

    if (!findField("lineScale")->isDefault())
      writer << " lineScale='" << mLineScale->value() << "'";

    if (!findField("window")->isDefault())
      writer << " window='" << mWindow->value() << "'";
  } else
    WbBaseNode::exportNodeFields(writer);
}
