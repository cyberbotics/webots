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
space :=
space +=
WEBOTS_HOME_PATH=$(subst $(space),\ ,$(strip $(subst \,/,$(WEBOTS_HOME))))
include $(WEBOTS_HOME_PATH)/resources/Makefile.os.include

.PHONY: release debug profile clean

TARGETS = libraries.Makefile controllers/braitenberg.Makefile libraries/vehicle.Makefile resources.Makefile

ROS_MAKEFILE = controllers/ros.Makefile

ifneq ($(OSTYPE),windows)
TARGETS += $(ROS_MAKEFILE)
else
ifeq ($(MAKECMDGOALS),clean)
TARGETS += $(ROS_MAKEFILE)
else
ifneq (,$(wildcard /mingw64/include/boost))
TARGETS += $(ROS_MAKEFILE)
else
MESSAGE = '\043 \033[0;33mboost not installed, skipping ros controller\033[0m'
endif
endif
endif

release debug profile: $(TARGETS)
ifneq ($(MESSAGE),)
	+@echo -e $(MESSAGE)
endif

clean: $(TARGETS)
	@-rm -rf controllers/sumo_supervisor/*.pyc

controllers/%.Makefile libraries/%.Makefile: libraries.Makefile controllers/ros/include.Makefile
	+@echo "# make" $(MAKECMDGOALS) $(@:.Makefile=)
	+@make -s -C $(@:.Makefile=) $(MAKECMDGOALS)

 %.Makefile controllers/ros/%.Makefile:
	+@echo "# make" $(MAKECMDGOALS) $(@:.Makefile=)
	+@make -s -C $(@:.Makefile=) $(MAKECMDGOALS)

resources.Makefile:
	@echo "#"; echo "# *** resources ***"
	+@make -s -C resources $(MAKECMDGOALS)
