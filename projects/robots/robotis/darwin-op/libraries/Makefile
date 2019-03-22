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

TARGETS=robotis-op2.Makefile managers.Makefile python.Makefile

ifeq ($(OSTYPE),windows)
PYTHON_MAKE = PATH="$(PYTHON27_HOME):$(PATH)" make
else
PYTHON_MAKE = make
endif

.PHONY: release debug profile clean

release debug profile clean: $(TARGETS)

managers.Makefile: robotis-op2.Makefile

managers.Makefile robotis-op2.Makefile:
	+@echo "# make" $(MAKECMDGOALS) $(@:.Makefile=)
	+@make -s -C $(@:.Makefile=) $(MAKECMDGOALS)

python.Makefile: managers.Makefile
	+@echo "# make" $(MAKECMDGOALS) $(@:.Makefile=)
	+@$(PYTHON_MAKE) -s -C $(@:.Makefile=) $(MAKECMDGOALS)
