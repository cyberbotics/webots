// Copyright 1996-2021 Cyberbotics Ltd.
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

#ifndef WB_MAC_ADDRESS_HPP
#define WB_MAC_ADDRESS_HPP

//
// Description: read the MAC addresses of the network cards of the computer
// (needed to detect if we are running on a Virtual Machine to display warnings about OpenGL support)
//

#include <QtCore/QString>

class WbMacAddress {
public:
  static WbMacAddress *instance();
  WbMacAddress();
  bool check(const QString &macAddress) const;
  QString address() const;
  const QString &error() const { return mError; }

private:
  unsigned char mAddress[6];
  QString mError;
};

#endif  // WB_MAC_ADDRESS_HPP
