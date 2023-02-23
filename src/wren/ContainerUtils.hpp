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

#ifndef CONTAINER_UTILS_HPP
#define CONTAINER_UTILS_HPP

#include <vector>

namespace wren {

  namespace containerutils {

    template<class T> void removeElementFromVector(std::vector<T> &v, const T &element) {
      if (v.empty())
        return;
      else if (v.back() == element)
        v.pop_back();
      else {
        for (typename std::vector<T>::iterator it = v.end() - 1; it >= v.begin(); --it) {
          if (*it == element) {
            *it = v.back();
            v.pop_back();

            return;
          }
        }
      }
    }

  }  // namespace containerutils
}  // namespace wren

#endif  // CONTAINER_UTILS_HPP
