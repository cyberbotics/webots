# Copyright 1996-2019 Cyberbotics Ltd.
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
  gpu_picker.js \
  util.js \
  dialog_window.js \
  console.js \
  help_window.js \
  robot_window.js \
  toolbar.js \
  editor.js \
  context_menu.js \
  viewpoint.js \
  selector.js \
  mouse_events.js \
  texture_loader.js \
  default_url.js \
  server.js \
  stream.js \
  video.js \
  x3d.js \
  x3d_scene.js \
  animation.js \
  webots.js

BABEL_JS_SOURCES = \
  node_modules/@babel/polyfill/dist/polyfill.js

# Token file used to determine when the last successful installation occurred.
INSTALL_TOKEN = install.token

.PHONY: release debug profile update cleanse clean

NPM_EXISTS = $(shell which npm 2> /dev/null)

ifeq ($(NPM_EXISTS),)
release debug profile update:
	@echo "# Please install nodejs to build 'webots.min.js' using the instructions on the GitHub wiki."
else
release debug profile update: webots.min.js
endif

webots.min.js: $(JS_SOURCES) $(INSTALL_TOKEN)
	@echo "# compressing webots.min.js"
	@node_modules/.bin/babel $(JS_SOURCES) $(BABEL_JS_SOURCES) -o webots.max.js --presets "@babel/preset-env"
	@node_modules/.bin/minify webots.max.js -o webots.min.js
	@rm webots.max.js

$(INSTALL_TOKEN): package.json
	@echo "# installing npm dependencies"
	@npm --silent install 1> /dev/null
	@touch $(INSTALL_TOKEN)

cleanse: clean
	@echo "# uninstalling npm dependencies"
	@npm --silent uninstall 1> /dev/null
	@rm -fr node_modules $(INSTALL_TOKEN)

clean:
	@rm -f *.min.js
