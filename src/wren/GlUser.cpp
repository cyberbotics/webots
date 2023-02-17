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

#include "GlUser.hpp"

#include "Debug.hpp"

#include <algorithm>

namespace wren {

  std::vector<std::pair<GlUser *, GlUser::GlAction>> GlUser::cUsersRequiringAction;

  void GlUser::setRequireAction(GlAction action) {
    assert(std::find_if(cUsersRequiringAction.cbegin(), cUsersRequiringAction.cend(),
                        [&](const std::pair<GlUser *, GlAction> &p) { return p.first == this && p.second == action; }) ==
           cUsersRequiringAction.cend());
    cUsersRequiringAction.push_back(std::make_pair(this, action));
  }

  void GlUser::applyGl() {
    for (size_t i = 0; i < cUsersRequiringAction.size(); ++i) {
      if (cUsersRequiringAction[i].second == GL_ACTION_CLEANUP)
        cUsersRequiringAction[i].first->cleanupGl();
      else if (cUsersRequiringAction[i].second == GL_ACTION_DELETE) {
        cUsersRequiringAction[i].first->cleanupGl();

        // make sure prepareGl doesn't get called on deleted users
        auto it =
          std::find_if(cUsersRequiringAction.begin(), cUsersRequiringAction.end(), [&](const std::pair<GlUser *, GlAction> &p) {
            return p.first == cUsersRequiringAction[i].first && p.second == GL_ACTION_PREPARE;
          });

        if (it != cUsersRequiringAction.end())
          it->first = NULL;

        delete cUsersRequiringAction[i].first;
      } else if (cUsersRequiringAction[i].second == GL_ACTION_PREPARE) {
        if (cUsersRequiringAction[i].first)
          cUsersRequiringAction[i].first->prepareGl();
      }
    }

    cUsersRequiringAction.clear();
  }

}  // namespace wren
