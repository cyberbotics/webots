# Copyright 1996-2018 Cyberbotics Ltd.
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

.PHONY: release debug profile clean c/car c/driver cpp/car cpp/driver python java

release debug profile clean: java python

java: cpp/driver cpp/car
	+@echo "# make" $(MAKECMDGOALS) $@
	+@make -s -C $@ $(MAKECMDGOALS)

python: cpp/driver cpp/car
ifeq ($(OSTYPE), windows)
	+@echo "# make" $(MAKECMDGOALS) python 3.7
	+@make -s -C python $(MAKECMDGOALS)
	+@echo "# make" $(MAKECMDGOALS) python 2.7
	+@PATH="$(PYTHON27_HOME):$(PATH)" make -s -C python $(MAKECMDGOALS)
endif
ifeq ($(OSTYPE),linux)
	+@echo "# make" $(MAKECMDGOALS) python 3.6
	+@PYTHON_COMMAND=python3.6 make -s -C python $(MAKECMDGOALS)
	@echo "# make" $(MAKECMDGOALS) python 2.7;
	+@PYTHON_COMMAND=python2.7 make -s -C python $(MAKECMDGOALS)
ifeq ($(UBUNTU_VERSION), 16.04)
	@echo "# make" $(MAKECMDGOALS) python 3.5;
	@+PYTHON_COMMAND=python3.5 make -s -C python $(MAKECMDGOALS)
endif
endif
ifeq ($(OSTYPE),darwin)
	+@echo "# make" $(MAKECMDGOALS) python 3.7
	+@PYTHON_COMMAND=python3.7 make -s -C python $(MAKECMDGOALS)
	@echo "# make" $(MAKECMDGOALS) python 2.7;
	+@PYTHON_COMMAND=python2.7 make -s -C python $(MAKECMDGOALS)
endif


cpp/driver: c/driver
	+@echo "# make" $(MAKECMDGOALS) $@
	+@make -s -C $@ $(MAKECMDGOALS)

cpp/car c/driver: c/car
	+@echo "# make" $(MAKECMDGOALS) $@
	+@make -s -C $@ $(MAKECMDGOALS)

c/car:
	+@echo "# make" $(MAKECMDGOALS) $@
	+@make -s -C c/car $(MAKECMDGOALS)
