# Copyright 1996-2023 Cyberbotics Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

.PHONY: api cache other_api parser physics protos rendering with_rendering

release debug profile: api cache other_api parser physics protos rendering with_rendering

clean: api cache parser physics protos rendering
	@-rm -rf *.txt \
	         *.pyc \
	         api/controllers/supervisor_animation/animation.html \
	         api/controllers/supervisor_animation/animation.json \
	         api/controllers/supervisor_animation/animation.x3d \
	         api/controllers/supervisor_animation/animation.css \
	         api/controllers/supervisor_animation/textures \
	         api/controllers/supervisor_export_image/image.jpg \
	         api/controllers/supervisor_export_image/image.png \
	         api/controllers/supervisor_save_world/test.wbt \
	         api/controllers/supervisor_start_stop_movie/movie.mp4
	@find . \( -name *.txt ! -name *expected_*.txt ! -path "./default/*" ! -path "./sources/*" \) | xargs rm -f
	@find . -name *.pyc | xargs rm -f
	@find . -name *.dif | xargs rm -f
	@find . -name *.tif | xargs rm -f
	@find . -name *_temp.wbt | xargs rm -f
	@find . \( -name TestSuite*.proto ! -path "./default/*" \) | xargs rm -f
	@find . \( -name test_suite_supervisor ! -path "./default/*" \) | xargs rm -rf

api:
	+@make -s -C api $(MAKECMDGOALS)

cache:
	+@make -s -C cache $(MAKECMDGOALS)

other_api:
	+@make -s -C other_api $(MAKECMDGOALS)

parser:
	+@make -s -C parser $(MAKECMDGOALS)

physics:
	+@make -s -C physics $(MAKECMDGOALS)

protos:
	+@make -s -C protos $(MAKECMDGOALS)

rendering:
	+@make -s -C rendering $(MAKECMDGOALS)

with_rendering:
	+@make -s -C with_rendering $(MAKECMDGOALS)
