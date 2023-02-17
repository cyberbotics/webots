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

#include "WbWrenVertexArrayFrameListener.hpp"

#include "WbMuscle.hpp"
#include "WbSimulationState.hpp"
#include "WbTrack.hpp"

#include <wren/scene.h>

WbWrenVertexArrayFrameListener *WbWrenVertexArrayFrameListener::cInstance = NULL;

static void processEvent() {
  static double lastUpdateTime = -1.0;
  const double currentTime = WbSimulationState::instance()->time();
  if (currentTime == lastUpdateTime)
    return;
  lastUpdateTime = currentTime;
  WbWrenVertexArrayFrameListener::instance()->frameStarted();
}

WbWrenVertexArrayFrameListener *WbWrenVertexArrayFrameListener::instance() {
  if (!cInstance)
    cInstance = new WbWrenVertexArrayFrameListener();

  return cInstance;
}

void WbWrenVertexArrayFrameListener::clear() {
  delete cInstance;
  cInstance = NULL;
}

WbWrenVertexArrayFrameListener::WbWrenVertexArrayFrameListener() : mListening(false) {
}

WbWrenVertexArrayFrameListener::~WbWrenVertexArrayFrameListener() {
  mTrackList.clear();
  mMuscleList.clear();
  if (mListening)
    wr_scene_remove_frame_listener(wr_scene_get_instance(), &processEvent);
}

void WbWrenVertexArrayFrameListener::subscribeTrack(WbTrack *track) {
  if (mTrackList.contains(track))
    return;
  mTrackList.append(track);
  updateListening();
}

void WbWrenVertexArrayFrameListener::subscribeMuscle(WbMuscle *muscle) {
  if (mMuscleList.contains(muscle))
    return;
  mMuscleList.append(muscle);
  updateListening();
}

void WbWrenVertexArrayFrameListener::unsubscribeTrack(WbTrack *track) {
  mTrackList.removeAll(track);
  updateListening();
}

void WbWrenVertexArrayFrameListener::unsubscribeMuscle(WbMuscle *muscle) {
  mMuscleList.removeAll(muscle);
  updateListening();
}

void WbWrenVertexArrayFrameListener::frameStarted() {
  for (int i = 0; i < mTrackList.size(); ++i)
    mTrackList[i]->animateMesh();
  mTrackList.clear();
  for (int i = 0; i < mMuscleList.size(); ++i)
    mMuscleList[i]->animateMesh();
  mMuscleList.clear();
}

void WbWrenVertexArrayFrameListener::updateListening() {
  bool need = mMuscleList.size() > 0 || mTrackList.size() > 0;
  if (need && !mListening) {
    wr_scene_add_frame_listener(wr_scene_get_instance(), &processEvent);
    mListening = true;
  } else if (!need && mListening) {
    wr_scene_remove_frame_listener(wr_scene_get_instance(), &processEvent);
    mListening = false;
  }
}
