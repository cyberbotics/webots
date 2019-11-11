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

#ifndef WB_ABSTRACT_CAMERA_HPP
#define WB_ABSTRACT_CAMERA_HPP

#include "WbRenderingDevice.hpp"
#include "WbSensor.hpp"

struct WrTransform;
struct WrStaticMesh;
struct WrRenderable;
struct WrMaterial;

class WbLens;
class WbWrenCamera;

#ifdef _WIN32
class QSharedMemory;
#else
class WbPosixSharedMemory;
#endif

class QDataStream;

class WbAbstractCamera : public WbRenderingDevice {
  Q_OBJECT

public:
  // constructors and destructor
  WbAbstractCamera(const QString &modelName, WbTokenizer *tokenizer = NULL);
  WbAbstractCamera(const WbAbstractCamera &other);
  WbAbstractCamera(const WbNode &other);
  virtual ~WbAbstractCamera();

  // reimplemented public functions
  void createWrenObjects() override;
  void preFinalize() override;
  void postFinalize() override;
  void writeAnswer(QDataStream &) override;
  void writeConfigure(QDataStream &) override;
  void reset() override;

  void updateCameraTexture();

  void setNodeVisibility(WbBaseNode *node, bool visible);

  bool isEnabled() { return mSensor ? mSensor->isEnabled() : false; }

  // external window
  void enableExternalWindow(bool enabled) override;
  virtual bool isRangeFinder() { return false; }
  bool spherical() const { return mSpherical->value(); }
  virtual double minRange() const = 0;
  virtual double maxRange() const { return 1.0; }
  virtual double nearValue() const { return mNear->value(); }  // near is a reserved keyword on Windows
  virtual double fieldOfView() const { return mFieldOfView->value(); }

  void resetSharedMemory();

  // static functions
  static int cCameraNumber;
  static int cCameraCounter;
  static void resetStaticCounters() { cCameraNumber = 0; }

  const unsigned char *constImage() const { return image(); }

signals:
  void enabled(WbAbstractCamera *camera, bool isActive);

protected:
  void setup() override;

  // user accessible fields
  WbSFDouble *mFieldOfView;
  WbSFBool *mSpherical;
  WbSFDouble *mNear;
  WbSFDouble *mMotionBlur;
  WbSFDouble *mNoise;
  WbSFNode *mLens;

  // private functions
  virtual void addConfigureToStream(QDataStream &stream, bool reconfigure = false);
  bool handleCommand(QDataStream &stream, unsigned char command);

  unsigned char *image() const { return mImageData; }
  WbLens *lens() const;

  virtual WbRgb enabledCameraFrustrumColor() const = 0;
  virtual WbRgb disabledCameraFrustrumColor() const { return WbRgb(0.5f, 0.5f, 0.5f); }

  void init();
  virtual void initializeSharedMemory();
  virtual void computeValue();
  void copyImageToSharedMemory();

  virtual bool antiAliasing() const { return false; }

  virtual int size() const = 0;

  // Wren methods
  virtual void createWrenCamera();
  void createWrenOverlay() override;
  void deleteWren();
  void applyNearToWren();
  virtual bool isFrustumEnabled() const { return false; }
  void applyFieldOfViewToWren();
  void applyMotionBlurToWren();
  void applyNoiseToWren();

  // WREN data
  WbWrenCamera *mWrenCamera;

  WrTransform *mTransform;
  WrRenderable *mRenderable;
  WrStaticMesh *mMesh;
  WrMaterial *mMaterial;

  // Frustum display
  WrTransform *mFrustumDisplayTransform;
  WrRenderable *mFrustumDisplayRenderable;
  WrStaticMesh *mFrustumDisplayMesh;
  WrMaterial *mFrustumDisplayMaterial;

  QList<WbBaseNode *> mInvisibleNodes;

  // other stuff
  WbSensor *mSensor;
  short mRefreshRate;
#ifdef _WIN32
  QSharedMemory *mImageShm;
#else
  WbPosixSharedMemory *mImageShm;
#endif
  unsigned char *mImageData;
  char mCharType;
  bool mNeedToConfigure;
  bool mHasSharedMemoryChanged;
  bool mImageReady;
  bool mImageChanged;

  bool mNeedToCheckShaderErrors;

  bool mSharedMemoryReset;

  bool mExternalWindowEnabled;
  void updateFrustumDisplay();
  void updateTextureUpdateNotifications();

public slots:
  void updateAntiAliasing();

protected slots:
  // update methods
  void updateWidth() override;
  void updateHeight() override;
  virtual void applyCameraSettingsToWren();
  virtual void applyFrustumToWren();
  virtual void updateOptionalRendering(int option);
  virtual void updateFieldOfView();
  void updateBackground();
  void updatePostProcessingEffect();
  void updateSpherical();
  void updateMotionBlur();
  void updateNoise();
  void updateLens();
  void applyLensToWren();
  void removeInvisibleNodeFromList(QObject *node);

  virtual void updateFrustumDisplayIfNeeded(int optionalRendering) {}

private:
  WbAbstractCamera &operator=(const WbAbstractCamera &);  // non copyable
};

#endif  // WB_ABSTRACT_CAMERA_HPP
