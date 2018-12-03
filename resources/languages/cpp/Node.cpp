// Copyright 1996-2018 Cyberbotics Ltd.
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

#define WB_ALLOW_MIXING_C_AND_CPP_API
#include <webots/supervisor.h>
#include <webots/Node.hpp>

#include <iostream>
#include <map>

using namespace std;
using namespace webots;

static map<WbNodeRef, Node *> nodeMap;

Node *Node::findNode(WbNodeRef ref) {
  if (!ref)
    return NULL;

  map<WbNodeRef, Node *>::iterator iter = nodeMap.find(ref);
  if (iter != nodeMap.end())
    return iter->second;

  Node *node = new Node(ref);
  nodeMap.insert(pair<WbNodeRef, Node *>(ref, node));
  return node;
}

void Node::cleanup() {
  map<WbNodeRef, Node *>::iterator iter;
  for (iter = nodeMap.begin(); iter != nodeMap.end(); ++iter)
    delete (*iter).second;

  nodeMap.clear();
}

Node::Node(WbNodeRef ref) {
  nodeRef = ref;
}

void Node::remove() {
  wb_supervisor_node_remove(nodeRef);
}

int Node::getId() const {
  return wb_supervisor_node_get_id(nodeRef);
}

Node::Type Node::getType() const {
  return Type(wb_supervisor_node_get_type(nodeRef));
}

std::string Node::getDef() const {
  return string(wb_supervisor_node_get_def(nodeRef));
}

std::string Node::getTypeName() const {
  return string(wb_supervisor_node_get_type_name(nodeRef));
}

std::string Node::getBaseTypeName() const {
  return string(wb_supervisor_node_get_base_type_name(nodeRef));
}

Node *Node::getParentNode() const {
  WbNodeRef parentRef = wb_supervisor_node_get_parent_node(nodeRef);
  return findNode(parentRef);
}

Field *Node::getField(const std::string &fieldName) const {
  WbFieldRef fieldRef = wb_supervisor_node_get_field(nodeRef, fieldName.c_str());
  return Field::findField(fieldRef);
}

const double *Node::getPosition() const {
  return wb_supervisor_node_get_position(nodeRef);
}

const double *Node::getOrientation() const {
  return wb_supervisor_node_get_orientation(nodeRef);
}

const double *Node::getCenterOfMass() const {
  return wb_supervisor_node_get_center_of_mass(nodeRef);
}

const double *Node::getContactPoint(int index) const {
  return wb_supervisor_node_get_contact_point(nodeRef, index);
}

int Node::getNumberOfContactPoints() const {
  return wb_supervisor_node_get_number_of_contact_points(nodeRef);
}

bool Node::getStaticBalance() const {
  return wb_supervisor_node_get_static_balance(nodeRef);
}

const double *Node::getVelocity() const {
  return wb_supervisor_node_get_velocity(nodeRef);
}

void Node::setVelocity(const double velocity[6]) {
  wb_supervisor_node_set_velocity(nodeRef, velocity);
}

void Node::resetPhysics() {
  wb_supervisor_node_reset_physics(nodeRef);
}

void Node::restartController() {
  wb_supervisor_node_restart_controller(nodeRef);
}

void Node::setVisibility(Node *from, bool visible) {
  wb_supervisor_node_set_visibility(nodeRef, from->nodeRef, visible);
}

void Node::moveViewpoint() const {
  wb_supervisor_node_move_viewpoint(nodeRef);
}
