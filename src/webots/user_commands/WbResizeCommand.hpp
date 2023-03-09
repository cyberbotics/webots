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

#ifndef WB_RESIZE_COMMAND_HPP
#define WB_RESIZE_COMMAND_HPP

//
// Description: Representation of 'resize' and 'rescale' actions on geometries
//              and definition of respective undo and redo functions
//

#include "WbVector3.hpp"

#include <QtGui/QUndoCommand>

class WbGeometry;

class WbResizeCommand : public QUndoCommand {
public:
  WbResizeCommand(WbGeometry *geometry, const WbVector3 &scale, QUndoCommand *parent = 0);
  ~WbResizeCommand() {}

  void undo() override;
  void redo() override;

protected:
  bool mIsFirstCall;
  WbGeometry *mGeometry;
  const WbVector3 mScale;
  const WbVector3 mInvScale;

private:
  void resetValue(const WbVector3 &scale);
};

#endif
