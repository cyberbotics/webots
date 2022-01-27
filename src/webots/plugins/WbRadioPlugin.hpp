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

#ifndef WB_RADIO_PLUGIN_HPP
#define WB_RADIO_PLUGIN_HPP

#include "WbPlugin.hpp"

class WbRadioPlugin : public WbPlugin {
  Q_OBJECT

public:
  // singleton instance
  static WbRadioPlugin *instance();

  static void loadAndInit(const QString &pluginName);
  static void cleanupAndUnload();

  // user implemented functions in shared library
  void init() { (*((void (*)())mFunctions[0]))(); }
  void cleanup() { (*((void (*)())mFunctions[1]))(); }
  int newRadio() { return (*((int (*)())mFunctions[2]))(); }
  void deleteRadio(int id) { (*((void (*)(int))mFunctions[3]))(id); }
  void setProtocol(int id, const char *prot) { (*((void (*)(int, const char *))mFunctions[4]))(id, prot); }
  void setAddress(int id, const char *addr) { (*((void (*)(int, const char *))mFunctions[5]))(id, addr); }
  void setFrequency(int id, double hz) { (*((void (*)(int, double))mFunctions[6]))(id, hz); }
  void setChannel(int id, int channel) { (*((void (*)(int, int))mFunctions[7]))(id, channel); }
  void setBitrate(int id, double baud) { (*((void (*)(int, double))mFunctions[8]))(id, baud); }
  void setRxSensitivity(int id, double dBm) { (*((void (*)(int, double))mFunctions[9]))(id, dBm); }
  void setTxPower(int id, double dBm) { (*((void (*)(int, double))mFunctions[10]))(id, dBm); }
  void move(int id, double x, double y, double z) { (*((void (*)(int, double, double, double))mFunctions[11]))(id, x, y, z); }
  void send(int id, const char *dest, const void *data, int size, double delay) {
    (*((void (*)(int, const char *, const void *, int, double))mFunctions[12]))(id, dest, data, size, delay);
  }
  void setCallback(int id, void *userData, void (*callback)(int, const struct WebotsRadioEvent *)) {
    (*((void (*)(int, void *, void (*)(int, const struct WebotsRadioEvent *)))mFunctions[13]))(id, userData, callback);
  }
  void run(double sec) { (*((void (*)(double))mFunctions[14]))(sec); }

  // constructor & destructor
  // name: name of the plugin, e.g. "omnet"
  explicit WbRadioPlugin(const QString &name);
  virtual ~WbRadioPlugin();

protected:
  // reimplemented protected functions
  const QString &type() const override {
    static const QString TYPE("radio");
    return TYPE;
  }
  int functionCount() const override;
  const char *functionName(int index) const override;
  bool shouldCopyBeforeLoad() const override { return true; }
};

#endif
