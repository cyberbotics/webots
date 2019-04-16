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
  texture_manager.js \
  resource_manager.js \
  server.js \
  stream.js \
  video_manager.js \
  x3d.js \
  x3d_scene_manager.js \
  animation.js \
  webots.js

NPM_PACKAGES = \
  terser \
  @babel/core \
	@babel/cli \
	@babel/preset-env

release debug profile update: install $(JS_SOURCES)
	@echo "# Compressing webots.js.min"
	@node_modules/.bin/babel $(JS_SOURCES) -o webots.max.js --presets "@babel/preset-env"
	@node_modules/.bin/terser webots.max.js -o webots.min.js
	@rm webots.max.js

install:
	npm install $(NPM_PACKAGES)

uninstall:
	npm uninstall $(NPM_PACKAGES)

clean:
	@rm -f *.min.js
