# Copyright 1996-2020 Cyberbotics Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

JS_SOURCES = \
	my_js_lib.js\
	my_renderer.js\
  util.js \
  system_info.js \
  dialog_window.js \
  console.js \
  help_window.js \
  robot_window.js \
  toolbar.js \
  editor.js \
  context_menu.js \
  viewpoint.js \
  selector.js \
  mouse_events.js\
  multimedia_client.js \
  texture_loader.js \
  default_url.js \
  server.js \
  stream.js \
  x3d.js \
  x3d_scene.js \
  animation.js \
  webots.js

BABEL_JS_SOURCES = \
  node_modules/@babel/polyfill/dist/polyfill.js



DEPENDENCIES_PATH = dependencies

null :=
space := $(null) $(null)
WEBOTS_HOME_PATH=$(subst $(space),\ ,$(strip $(subst \,/,$(WEBOTS_HOME))))
include $(WEBOTS_HOME_PATH)/resources/Makefile.os.include

ifeq ($(OSTYPE),darwin)
 WGET=LANG=en_US.UTF-8 wget -qN -O
 DISABLE_NPM_INSTALL_WARNING=2>&1
else
 WGET=wget -qN -O
 DISABLE_NPM_INSTALL_WARNING=
endif


# Token file used to determine when the last successful installation occurred.
INSTALL_TOKEN = install.token

.PHONY: release debug profile update cleanse clean

NPM_EXISTS = $(shell which npm 2> /dev/null)

ifeq ($(NPM_EXISTS),)
release debug profile update:
	@echo "# npm not found in PATH:" $(PATH)
	@echo "# Please install nodejs to build 'webots.min.js' using the instructions on the GitHub wiki."
	@echo -e "# \033[0;33mnpm not installed, skipping webots.min.js\033[0m"
else
NPM_INSTALLED_PACKAGES := $(shell npm list --parseable | grep "node_modules" | sed 's/^.*node_modules\///')
release profile update: webots.min.js webots.lib.js
debug: webots.debug.js
endif

webots.min.js: $(JS_SOURCES) $(INSTALL_TOKEN)
	@echo "# compressing webots.min.js"
	@node_modules/.bin/babel $(JS_SOURCES) $(BABEL_JS_SOURCES) -o webots.tmp1.js --presets "@babel/preset-env"
	@cat webots.tmp1.js > webots.tmp2.js
	@node_modules/.bin/minify webots.tmp2.js -o $@
	@rm webots.tmp2.js webots.tmp1.js

webots.debug.js: $(JS_SOURCES) $(INSTALL_TOKEN)
	@echo "# creating webots.debug.js"
	@cat $(JS_SOURCES) $(BABEL_JS_SOURCES) > $@

# JS module intended to be included in another module.
# webots.lib.js is not processed with babel or minify
webots.lib.js: $(JS_SOURCES) $(INSTALL_TOKEN)
	@echo "# compressing webots.lib.js"
	@cat $(JS_SOURCES) > $@


$(INSTALL_TOKEN): package.json
	@echo "# installing npm dependencies"
	@npm --silent install 1> /dev/null $(DISABLE_NPM_INSTALL_WARNING)
	@touch $(INSTALL_TOKEN)

cleanse: clean
	@echo "# uninstalling npm dependencies"
ifneq ($(NPM_INSTALLED_PACKAGES),)
	@npm --silent --no-save uninstall $(NPM_INSTALLED_PACKAGES)
endif
	@rm -fr node_modules $(INSTALL_TOKEN) package-lock.json

clean:
	@rm -fr *.min.js webots.debug.js webots.lib.js $(DEPENDENCIES_PATH)
