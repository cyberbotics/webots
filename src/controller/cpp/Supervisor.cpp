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

#define WB_ALLOW_MIXING_C_AND_CPP_API
#include <stdio.h>
#include <webots/supervisor.h>
#include <webots/Supervisor.hpp>

using namespace std;
using namespace webots;

Supervisor::~Supervisor() {
  Field::cleanup();
  Node::cleanup();
}

Supervisor *Supervisor::getSupervisorInstance() {
  if (cInstance) {
    if (!dynamic_cast<Supervisor *>(cInstance)) {
      cerr << "A Robot instance is already present, it cannot be casted to Supervisor" << endl;
      return NULL;
    }
    return static_cast<Supervisor *>(cInstance);
  }
  cInstance = new Supervisor();
  return static_cast<Supervisor *>(cInstance);
}

void Supervisor::simulationQuit(int status) {
  wb_supervisor_simulation_quit(status);
}

void Supervisor::simulationRevert() {
  fprintf(stderr, "Supervisor::simulationRevert is deprecated, please use Supervisor::worldReload instead\n");
  wb_supervisor_world_reload();
}

void Supervisor::simulationPhysicsReset() {
  fprintf(stderr, "Supervisor::simulationPhysicsReset is deprecated, please use Supervisor::simulationResetPhysics instead\n");
  wb_supervisor_simulation_reset_physics();
}

void Supervisor::simulationResetPhysics() {
  wb_supervisor_simulation_reset_physics();
}

void Supervisor::simulationReset() {
  wb_supervisor_simulation_reset();
}

Supervisor::SimulationMode Supervisor::simulationGetMode() const {
  return SimulationMode(wb_supervisor_simulation_get_mode());
}

void Supervisor::simulationSetMode(SimulationMode mode) {
  wb_supervisor_simulation_set_mode(WbSimulationMode(mode));
}

void Supervisor::loadWorld(const std::string &file) {
  fprintf(stderr, "Supervisor::loadWorld is deprecated, please use Supervisor::worldLoad instead\n");
  wb_supervisor_world_load(file.c_str());
}

void Supervisor::worldLoad(const std::string &file) {
  wb_supervisor_world_load(file.c_str());
}

void Supervisor::worldReload() {
  wb_supervisor_world_reload();
}

bool Supervisor::saveWorld() {
  fprintf(stderr, "Supervisor::saveWorld is deprecated, please use Supervisor::worldSave instead\n");
  return wb_supervisor_world_save(NULL);
}

bool Supervisor::worldSave() {
  return wb_supervisor_world_save(NULL);
}

bool Supervisor::saveWorld(const std::string &file) {
  fprintf(stderr, "Supervisor::saveWorld is deprecated, please use Supervisor::worldSave instead\n");
  return wb_supervisor_world_save(file.c_str());
}

bool Supervisor::worldSave(const std::string &file) {
  return wb_supervisor_world_save(file.c_str());
}

void Supervisor::exportImage(const string &file, int quality) const {
  wb_supervisor_export_image(file.c_str(), quality);
}

bool Supervisor::animationStartRecording(const string &file) {
  return wb_supervisor_animation_start_recording(file.c_str());
}

bool Supervisor::animationStopRecording() {
  return wb_supervisor_animation_stop_recording();
}

void Supervisor::startMovie(const string &file, int width, int height, int codec, int quality, int acceleration, bool caption) {
  fprintf(stderr, "Supervisor::startMovie is deprecated, please use Supervisor::movieStartRecording instead\n");
  wb_supervisor_movie_start_recording(file.c_str(), width, height, codec, quality, acceleration, caption);
}

void Supervisor::movieStartRecording(const string &file, int width, int height, int codec, int quality, int acceleration,
                                     bool caption) {
  wb_supervisor_movie_start_recording(file.c_str(), width, height, codec, quality, acceleration, caption);
}

void Supervisor::stopMovie() {
  fprintf(stderr, "Supervisor::stopMovie is deprecated, please use Supervisor::movieStopRecording instead\n");
  wb_supervisor_movie_stop_recording();
}

void Supervisor::movieStopRecording() {
  wb_supervisor_movie_stop_recording();
}

int Supervisor::getMovieStatus() {
  fprintf(stderr, "Supervisor::getMovieStatus is deprecated, please use Supervisor::movieGetStatus instead\n");
  return wb_supervisor_movie_get_status();
}

int Supervisor::movieGetStatus() const {
  return wb_supervisor_movie_get_status();
}

bool Supervisor::movieIsReady() const {
  return wb_supervisor_movie_is_ready();
}

bool Supervisor::movieFailed() const {
  return wb_supervisor_movie_failed();
}

void Supervisor::setLabel(int id, const string &label, double xpos, double ypos, double size, int color, double transparency,
                          const string &font) {
  wb_supervisor_set_label(id, label.c_str(), xpos, ypos, size, color, transparency, font.c_str());
}

Node *Supervisor::getRoot() const {
  WbNodeRef nodeRef = wb_supervisor_node_get_root();
  return Node::findNode(nodeRef);
}

Node *Supervisor::getSelf() const {
  WbNodeRef nodeRef = wb_supervisor_node_get_self();
  return Node::findNode(nodeRef);
}

Node *Supervisor::getFromDef(const std::string &name) const {
  WbNodeRef nodeRef = wb_supervisor_node_get_from_def(name.c_str());
  return Node::findNode(nodeRef);
}

Node *Supervisor::getFromId(int id) const {
  WbNodeRef nodeRef = wb_supervisor_node_get_from_id(id);
  return Node::findNode(nodeRef);
}

Node *Supervisor::getFromDevice(const Device *device) const {
  return getFromDeviceTag(device->getTag());
}

Node *Supervisor::getFromDeviceTag(int tag) const {
  WbNodeRef nodeRef = wb_supervisor_node_get_from_device(tag);
  return Node::findNode(nodeRef);
}

Node *Supervisor::getSelected() const {
  WbNodeRef nodeRef = wb_supervisor_node_get_selected();
  return Node::findNode(nodeRef);
}

bool Supervisor::virtualRealityHeadsetIsUsed() const {
  return wb_supervisor_virtual_reality_headset_is_used();
}

const double *Supervisor::virtualRealityHeadsetGetPosition() const {
  return wb_supervisor_virtual_reality_headset_get_position();
}

const double *Supervisor::virtualRealityHeadsetGetOrientation() const {
  return wb_supervisor_virtual_reality_headset_get_orientation();
}
