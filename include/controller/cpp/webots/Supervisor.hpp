// Copyright 1996-2024 Cyberbotics Ltd.
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

#ifndef SUPERVISOR_HPP
#define SUPERVISOR_HPP

#include <webots/Device.hpp>
#include <webots/Node.hpp>
#include <webots/Robot.hpp>

namespace webots {
  class Supervisor : public Robot {
  public:
    typedef enum { SIMULATION_MODE_PAUSE = 0, SIMULATION_MODE_REAL_TIME, SIMULATION_MODE_FAST } SimulationMode;

    Supervisor() : Robot() {}
    static Supervisor *getSupervisorInstance();
    virtual ~Supervisor();
    virtual void simulationQuit(int status);
    virtual void simulationReset();
    virtual void simulationResetPhysics();

    SimulationMode simulationGetMode() const;
    virtual void simulationSetMode(SimulationMode mode);

    virtual void worldLoad(const std::string &file);
    virtual void worldReload();
    virtual bool worldSave();
    virtual bool worldSave(const std::string &file);

    void exportImage(const std::string &file, int quality) const;

    virtual bool animationStartRecording(const std::string &file);
    virtual bool animationStopRecording();

    virtual void movieStartRecording(const std::string &file, int width, int height, int codec, int quality, int acceleration,
                                     bool caption);
    virtual void movieStopRecording();
    bool movieIsReady() const;
    bool movieFailed() const;

    virtual void setLabel(int id, const std::string &label, double xpos, double ypos, double size, int color,
                          double transparency = 0, const std::string &font = "Arial");
    Node *getRoot() const;
    Node *getSelf() const;
    Node *getFromDef(const std::string &name) const;
    Node *getFromId(int id) const;
    Node *getFromDevice(const Device *device) const;
    Node *getFromDeviceTag(int tag) const;
    Node *getSelected() const;

    bool virtualRealityHeadsetIsUsed() const;
    const double *virtualRealityHeadsetGetPosition() const;
    const double *virtualRealityHeadsetGetOrientation() const;

    // Deprecated functions

    // Deprecated since Webots R2018b
    virtual void simulationRevert();                  // please use worldReload() instead
    virtual void loadWorld(const std::string &file);  // please use worldLoad() instead
    virtual bool saveWorld();                         // please use worldSave() instead
    virtual bool saveWorld(const std::string &file);  // please use worldSave() instead

    // Deprecated since Webots 8.0.0, please use simulationResetPhysics() instead
    virtual void simulationPhysicsReset();

    // deprecated since webots 8.3.0: please use the wb_supervisor_movie_*() functions instead
    virtual void startMovie(const std::string &file, int width, int height, int codec, int quality, int acceleration,
                            bool caption);
    virtual void stopMovie();
    int getMovieStatus();

    // deprecated since Webots 8.4.0: please use movieIsReady() and movieFailed()
    enum { MOVIE_READY = 0, MOVIE_RECORDING, MOVIE_SAVING, MOVIE_WRITE_ERROR, MOVIE_ENCODING_ERROR, MOVIE_SIMULATION_ERROR };
    int movieGetStatus() const;
  };
}  // namespace webots

#endif  // SUPERVISOR_HPP
