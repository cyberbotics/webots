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
#define WbMemoryMappedFile QSharedMemory
#else
class WbPosixMemoryMappedFile;
#define WbMemoryMappedFile WbPosixMemoryMappedFile
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
  void writeAnswer(WbDataStream &) override;
  void writeConfigure(WbDataStream &) override;
  void reset(const QString &id) override;

  virtual void updateCameraTexture();

  void externControllerChanged() { mHasExternControllerChanged = true; }
  void newRemoteExternController() { mIsRemoteExternController = true; }
  void removeRemoteExternController() { mIsRemoteExternController = false; }
  void enableExternalWindowForAttachedCamera(bool enabled);

  void setNodesVisibility(QList<const WbBaseNode *> nodes, bool visible);

  virtual bool isEnabled() const { return mSensor ? mSensor->isEnabled() : false; }

  // external window
  void enableExternalWindow(bool enabled) override;
  virtual bool isRangeFinder() { return false; }
  bool isPlanarProjection() const { return mProjection->value() == "planar"; }
  virtual double minRange() const = 0;
  virtual double maxRange() const { return 1.0; }
  virtual double nearValue() const { return mNear->value(); }  // near is a reserved keyword on Windows
  virtual double fieldOfView() const { return mFieldOfView->value(); }

  virtual void resetMemoryMappedFile();

  // static functions
  static int cCameraNumber;
  static int cCameraCounter;
  static void resetStaticCounters() { cCameraNumber = 0; }

  const unsigned char *constImage() const { return image(); }

signals:
  void enabled(WbAbstractCamera *camera, bool isActive);

protected:
  void setup() override;
  virtual void render(){};
  virtual bool needToRender() const;

  // user accessible fields
  WbSFDouble *mFieldOfView;
  WbSFString *mProjection;
  WbSFDouble *mNear;
  WbSFDouble *mMotionBlur;
  WbSFDouble *mNoise;
  WbSFNode *mLens;

  // private functions
  virtual void addConfigureToStream(WbDataStream &stream, bool reconfigure = false);
  bool handleCommand(QDataStream &stream, unsigned char command);

  unsigned char *image() const { return mImageData; }
  WbLens *lens() const;

  virtual WbRgb enabledCameraFrustrumColor() const = 0;
  virtual WbRgb disabledCameraFrustrumColor() const { return WbRgb(0.5f, 0.5f, 0.5f); }

  void init();
  virtual void initializeImageMemoryMappedFile();
  WbMemoryMappedFile *initializeMemoryMappedFile(const QString &id = "");
  virtual void computeValue();
  void copyImageToMemoryMappedFile(WbWrenCamera *camera, unsigned char *data);
  void editChunkMetadata(WbDataStream &stream, int newImageSize);

  virtual bool antiAliasing() const { return false; }

  virtual int size() const = 0;

  void applyCameraSettings();

  // Wren methods
  virtual void createWrenCamera();
  void createWrenOverlay() override;
  void deleteWren();
  virtual bool isFrustumEnabled() const { return false; }
  virtual void applyNearToWren();
  virtual void applyFieldOfViewToWren();
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

  QList<const WbBaseNode *> mInvisibleNodes;

  // other stuff
  WbSensor *mSensor;
  short mRefreshRate;
  WbMemoryMappedFile *mImageMemoryMappedFile;
  unsigned char *mImageData;
  char mCharType;
  bool mNeedToConfigure;
  bool mSendMemoryMappedFile;
  bool mHasExternControllerChanged;
  bool mIsRemoteExternController;
  bool mImageChanged;

  bool mNeedToCheckShaderErrors;

  bool mMemoryMappedFileReset;

  bool mExternalWindowEnabled;
  void updateFrustumDisplay();
  virtual void updateTextureUpdateNotifications(bool enabled);

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
  void updateProjection();
  void updateBackground();
  void updatePostProcessingEffect();
  void updateMotionBlur();
  void updateNoise();
  void updateLens();
  virtual void applyLensToWren();
  void removeInvisibleNodeFromList(QObject *node);

  virtual void updateFrustumDisplayIfNeeded(int optionalRendering) {}

private:
  WbAbstractCamera &operator=(const WbAbstractCamera &);  // non copyable
};

#endif  // WB_ABSTRACT_CAMERA_HPP
