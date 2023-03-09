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

#ifndef GL_USER_HPP
#define GL_USER_HPP

#include "Constants.hpp"

#include <vector>

namespace wren {

  // Classes which would otherwise apply OpenGL state changes outside of the render loop
  // need to inherit from this class and override its virtual methods.
  class GlUser {
  public:
    enum GlAction { GL_ACTION_PREPARE, GL_ACTION_CLEANUP, GL_ACTION_DELETE };

    static void applyGl();

    void setRequireAction(GlAction action);

  protected:
    GlUser() {}
    virtual ~GlUser() {}

    virtual void cleanupGl() = 0;
    virtual void prepareGl() = 0;

  private:
    static std::vector<std::pair<GlUser *, GlUser::GlAction>> cUsersRequiringAction;
  };

}  // namespace wren

#endif  // GL_USER_HPP
