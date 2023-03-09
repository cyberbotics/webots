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

#include "WbRadioPlugin.hpp"
#include <cassert>

WbRadioPlugin *gRadioPluginInstance = NULL;

// exclamation mark (!) indicates mandatory mFunctions
static const char *FUNCTION_NAMES[] = {"!webots_radio_init",
                                       "!webots_radio_cleanup",
                                       "!webots_radio_new",
                                       "!webots_radio_delete",
                                       "!webots_radio_set_protocol",
                                       "!webots_radio_set_address",
                                       "!webots_radio_set_frequency",
                                       "!webots_radio_set_channel",
                                       "!webots_radio_set_bitrate",
                                       "!webots_radio_set_rx_sensitivity",
                                       "!webots_radio_set_tx_power",
                                       "!webots_radio_move",
                                       "!webots_radio_send",
                                       "!webots_radio_set_callback",
                                       "!webots_radio_run"};

WbRadioPlugin *WbRadioPlugin::instance() {
  return gRadioPluginInstance;
}

WbRadioPlugin::WbRadioPlugin(const QString &name) : WbPlugin(name) {
  gRadioPluginInstance = this;
}

WbRadioPlugin::~WbRadioPlugin() {
  gRadioPluginInstance = NULL;
}

int WbRadioPlugin::functionCount() const {
  return sizeof(FUNCTION_NAMES) / sizeof(char *);
}

const char *WbRadioPlugin::functionName(int index) const {
  return FUNCTION_NAMES[index];
}

void WbRadioPlugin::loadAndInit(const QString &pluginName) {
  assert(gRadioPluginInstance == NULL);

  // default case: "" (no plugin)
  if (pluginName.isEmpty())
    return;

  gRadioPluginInstance = new WbRadioPlugin(pluginName);
  if (!gRadioPluginInstance->load()) {
    // could not load plugin or could not find all functions
    delete gRadioPluginInstance;
    gRadioPluginInstance = NULL;
    return;
  }

  gRadioPluginInstance->init();
}

void WbRadioPlugin::cleanupAndUnload() {
  if (gRadioPluginInstance) {
    gRadioPluginInstance->cleanup();
    delete gRadioPluginInstance;
    gRadioPluginInstance = NULL;
  }
}
