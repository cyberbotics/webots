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

#ifndef WB_SIMULATION_STATE_HPP
#define WB_SIMULATION_STATE_HPP

//
// Description: current simulation state
//

#include <QtCore/QObject>

class WbSimulationState : public QObject {
  Q_OBJECT

public:
  // singleton
  static WbSimulationState *instance() { return cInstance ? cInstance : new WbSimulationState(); };

  // simulation mode
  enum Mode { NONE, PAUSE, STEP, REALTIME, FAST };
  void setMode(Mode mode);
  void setRendering(bool show);
  void undoMode() { setMode(mPreviousMode); }
  Mode mode() const { return mEnabled ? mMode : PAUSE; }
  Mode previousMode() const { return mEnabled ? mPreviousMode : PAUSE; }
  bool isPaused() const { return mMode == PAUSE; }
  bool isStep() const { return mMode == STEP; }
  bool isRealTime() const { return mMode == REALTIME; }
  bool isFast() const { return mMode == FAST; }
  bool isRendering() const { return mPerformRendering; }
  // pause/resume simulation for executing application dialogs
  void pauseSimulation();
  void resumeSimulation();

  // enabled
  void setEnabled(bool enabled);
  bool isEnabled() const { return mEnabled; }

  // simulation time
  double time() const { return mTime; }  // milliseconds
  void resetTime();
  void increaseTime(double dt);
  bool hasStarted() const { return mTime > 0.0; }

  // ray tracing
  void subscribeToRayTracing();
  void unsubscribeToRayTracing();
  bool isRayTracingEnabled() { return mRayTracingSubscribersCount != 0; }

signals:
  // the simulation mode has changed
  void modeChanged();
  void renderingStateChanged();
  void enabledChanged(bool);

  // steps execution
  void physicsStepStarted();
  void physicsStepEnded();
  void cameraRenderingStarted();

  void controllerReadRequestsCompleted();

  // ray tracing is enabled
  void rayTracingEnabled();

protected:
  WbSimulationState();
  virtual ~WbSimulationState();

private:
  static WbSimulationState *cInstance;
  Mode mMode, mPreviousMode;

  bool mPerformRendering;
  bool mEnabled;
  double mTime;

  // ray tracing
  int mRayTracingSubscribersCount;
};

#endif
