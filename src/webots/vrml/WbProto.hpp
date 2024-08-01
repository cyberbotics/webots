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

#ifndef WB_PROTO_HPP
#define WB_PROTO_HPP

//
// Description: class containing proto-specific information about a VRML node
//

#include <QtCore/QList>
#include <QtCore/QObject>
#include <QtCore/QString>

class WbField;
class WbNode;
class WbProtoModel;

class WbProto {

public:
	WbProto(WbProtoModel *model, WbNode *node) : mModel(model), mNode(node) {}
	WbProto(WbProto *other, WbNode *node);

	const QString &name() const;
  WbNode *node() const { return mNode; }
	WbProtoModel *model() const { return mModel; }

  WbProto *parent() const { return mParentProto; }
	void setParentProto(WbProto *parentProto) { mParentProto = parentProto; }

	QList<WbField *> &parameters() { return mParameters; }
	const QList<WbField *> &parameters() const { return mParameters; }
  int parameterIndex(const WbField *field) const;

private:
  WbNode *mNode;
	WbProto *mParentProto;
	WbProtoModel *mModel;
	QList<WbField *> mParameters;

};

#endif
