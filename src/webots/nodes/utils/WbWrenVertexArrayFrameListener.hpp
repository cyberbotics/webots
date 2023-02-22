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

#ifndef WB_WREN_VERTEX_ARRAY_FRAME_LISTENER
#define WB_WREN_VERTEX_ARRAY_FRAME_LISTENER

#include <QtCore/QList>

class WbMuscle;
class WbTrack;

class WbWrenVertexArrayFrameListener {
public:
  static WbWrenVertexArrayFrameListener *instance();
  static void clear();

  void subscribeTrack(WbTrack *track);
  void unsubscribeTrack(WbTrack *track);

  void subscribeMuscle(WbMuscle *muscle);
  void unsubscribeMuscle(WbMuscle *muscle);

  void frameStarted();

private:
  static WbWrenVertexArrayFrameListener *cInstance;

  WbWrenVertexArrayFrameListener();
  ~WbWrenVertexArrayFrameListener();

  void updateListening();

  QList<WbTrack *> mTrackList;
  QList<WbMuscle *> mMuscleList;

  bool mListening;
};

#endif
