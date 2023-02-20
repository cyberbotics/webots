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

#ifndef WB_WREN_PICKER_HPP
#define WB_WREN_PICKER_HPP

//
// Description: utility class to help picking objects
//
#include <WbMatrix4.hpp>
#include <WbVector3.hpp>

#include <wren/frame_buffer.h>
#include <wren/renderable.h>
#include <wren/texture.h>
#include <wren/texture_rtt.h>
#include <wren/viewport.h>

class WbWrenPicker {
public:
  // These constants are reserved unique IDs used by the handles
  static const int HANDLES_X_AXIS = 0x7FFFFFF1, HANDLES_Y_AXIS = 0x7FFFFFF2, HANDLES_Z_AXIS = 0x7FFFFFF3,
                   HANDLES_TRANSLATE = 0x7FFFFFF0, HANDLES_ROTATE = 0x7FFFFFF4, HANDLES_SCALE = 0x7FFFFFF8,
                   HANDLES_RESIZE = 0x7FFFFFFC;

  static void setPickable(WrRenderable *renderable, int uniqueId, bool pickable);

  WbWrenPicker();
  ~WbWrenPicker();

  bool pick(int x, int y);

  const WbVector3 &screenCoordinates() const { return mCoordinates; }

  const int &selectedId() const { return mSelectedId; }

  int pickedTranslateHandle() const { return mPickedTranslation; }
  int pickedRotateHandle() const { return mPickedRotation; }
  int pickedScaleHandle() const { return mPickedScale; }
  int pickedResizeHandle() const { return mPickedResize; }

private:
  void setup();
  void cleanup();
  bool hasSizeChanged();

  WrViewport *mViewport;
  WrFrameBuffer *mFrameBuffer;
  WrTextureRtt *mOutputTexture;

  int mWidth;
  int mHeight;

  WbVector3 mCoordinates;
  int mSelectedId;

  int mPickedTranslation;
  int mPickedRotation;
  int mPickedScale;
  int mPickedResize;
};

#endif
