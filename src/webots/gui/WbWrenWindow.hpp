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

#ifndef WB_WREN_WINDOW_HPP
#define WB_WREN_WINDOW_HPP

//
// Description: Qt native window for rendering WREN
//

#include <QtCore/QMutex>
#include <QtGui/QWindow>
#include <QtOpenGL/QOpenGLFunctions_3_3_Core>

class WbMultimediaStreamingServer;

struct WrTextureRtt;
struct WrFrameBuffer;

class WbWrenWindow : public QWindow {
  Q_OBJECT;

public:
  static WbWrenWindow *instance();
  static void flipAndScaleDownImageBuffer(const unsigned char *source, unsigned char *destination, int sourceWidth,
                                          int sourceHeight, int scaleDownFactor);

  explicit WbWrenWindow();
  virtual ~WbWrenWindow();

  virtual QSize minimumSize() const;
  QSize sizeHint() const;

  // return immediate screenshot
  QImage grabWindowBufferNow();

  // read pixels asynchronously (faster)
  void initVideoPBO();
  void completeVideoPBOProcessing(bool canceled);
  void requestGrabWindowBuffer();

  void updateWrenViewportDimensions();

  void blitMainFrameBufferToScreen();

  void setVideoStreamingServer(WbMultimediaStreamingServer *streamingServer);

public slots:
  virtual void renderLater();

signals:
  void videoImageReady(unsigned char *frame);
  void resized();

protected:
  virtual void initialize();
  virtual void renderNow(bool culling = true);
  virtual void resizeWren(int width, int height);
  bool event(QEvent *event) override;

private:
  static WbWrenWindow *cInstance;

  void processVideoPBO();
  void updateFrameBuffer();
  void readPixels(int width, int height, unsigned int format, void *buffer);

  bool mUpdatePending;
  unsigned char *mSnapshotBuffer;
  int mSnapshotBufferWidth;
  int mSnapshotBufferHeight;
  GLuint mVideoPBOIds[2];
  int mVideoWidth;
  int mVideoHeight;
  int mVideoPBOIndex;

  // proxy framebuffer for post-processing effects
  WrFrameBuffer *mWrenMainFrameBuffer;
  WrTextureRtt *mWrenMainFrameBufferTexture;
  WrTextureRtt *mWrenNormalFrameBufferTexture;
  WrTextureRtt *mWrenDepthFrameBufferTexture;

  WbMultimediaStreamingServer *mVideoStreamingServer;

private slots:
  void feedMultimediaStreamer();
};

#endif
