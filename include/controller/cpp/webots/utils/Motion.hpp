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

#ifndef MOTION_HPP
#define MOTION_HPP

#define WB_USING_CPP_API
#include <string>
#include "../../../c/webots/types.h"

namespace webots {
  class Motion {
  public:
    Motion(const std::string &fileName);
    bool isValid() const;
    virtual ~Motion();
    virtual void play();
    virtual void stop();
    int getDuration() const;
    int getTime() const;
    virtual void setTime(int time);
    virtual void setReverse(bool reverse);
    virtual void setLoop(bool loop);
    bool isOver() const;

  protected:
    WbMotionRef getMotionRef() const { return motionRef; }

  private:
    WbMotionRef motionRef;
  };
}  // namespace webots

#endif  // MOTION_HPP
