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

# clear out all suffixes for implicit rules to speed up Makefile

release debug profile update:
	@echo "# Compressing webots.js.min"
	terser util.js dialog_window.js console.js help_window.js robot_window.js \
	toolbar.js editor.js context_menu.js viewpoint.js selector.js mouse_events.js \
	texture_manager.js resource_manager.js server.js stream.js video_manager.js \
	x3d.js x3d_scene_manager.js animation.js webots.js -o webots.min.js

wb_*.ts: $(WEBOTS_SOURCES)
	@echo "# updating $@"
	@$(LUPDATE) $(LUPDATE_FLAGS) $(WEBOTS_SRC_PATH) -ts $@

clean:
	rm -f *.min.js
