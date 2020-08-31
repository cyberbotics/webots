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

include ../Makefile.os.include

.PHONY: release debug profile

release debug profile clean:
	@echo "# make" $@ cpp;
	+make -s -C cpp $@
	@echo "# make" $@ java;
	+make -s -C java $@
ifeq ($(OSTYPE),linux)
	@echo "# make" $@ python2.7;
	+PYTHON_COMMAND=python2.7 make -s -C python $@
ifeq ($(UBUNTU_VERSION), 16.04)
	@echo "# make" $@ python3.5;
	+PYTHON_COMMAND=python3.5 make -s -C python $@
endif
	@echo "# make" $@ python3.6;
	+PYTHON_COMMAND=python3.6 make -s -C python $@
	@echo "# make" $@ python3.7;
	+PYTHON_COMMAND=python3.7 make -s -C python $@
	@echo "# make" $@ python3.8;
	+PYTHON_COMMAND=python3.8 make -s -C python $@
endif
ifeq ($(OSTYPE),windows)
	@echo "# make" $@ python3.8;
	+make -s -C python $@
	@echo "# make" $@ python3.7;
	+PATH="$(PYTHON37_HOME):$(PATH)" make -s -C python $@
	@echo "# make" $@ python2.7;
	+PATH="$(PYTHON27_HOME):$(PATH)" make -s -C python $@
endif
ifeq ($(OSTYPE),darwin)
	@echo "# make" $@ python2.7;
	+PYTHON_COMMAND=/usr/bin/python2.7 make -s -C python $@
	@echo "# make" $@ python3.7;
	+PYTHON_COMMAND=/Library/Frameworks/Python.framework/Versions/3.7/bin/python3.7 make -s -C python $@
	@echo "# make" $@ python3.8;
	+PYTHON_COMMAND=/Library/Frameworks/Python.framework/Versions/3.8/bin/python3.8 make -s -C python $@
	@echo "# make" $@ python3.7 brew;
	+PYTHON_COMMAND=/usr/local/opt/python@3.7/bin/python3.7 SUFFIX=_brew make -s -C python $@
	@echo "# make" $@ python3.8 brew;
	+PYTHON_COMMAND=/usr/local/opt/python@3.8/bin/python3.8 SUFFIX=_brew make -s -C python $@
endif
