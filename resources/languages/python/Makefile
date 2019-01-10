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

ifeq ($(WEBOTS_HOME),)
WEBOTS_HOME_PATH = ../../..
else
space :=
space +=
WEBOTS_HOME_PATH=$(subst $(space),\ ,$(strip $(subst \,/,$(WEBOTS_HOME))))
endif

include $(WEBOTS_HOME_PATH)/resources/Makefile.os.include
PYTHON_COMMAND ?= python
PYTHON_VERSION := $(shell $(PYTHON_COMMAND) --version 2>&1 | sed 's/.* \([0-9]\).\([0-9]\).*/\1.\2/')
PYTHON_SHORT_VERSION = $(subst .,,$(PYTHON_VERSION))
INTERFACE       = controller.i
SWIG            = swig
SWIG_OPTS       = -c++ -python -outdir $(WEBOTS_HOME_PATH)/lib/python$(PYTHON_SHORT_VERSION)/
WEBOTS_INCLUDES = -I$(WEBOTS_HOME_PATH)/include/controller/cpp -I$(WEBOTS_HOME_PATH)/include/controller/c
WRAPPER         = $(INTERFACE:.i=$(PYTHON_SHORT_VERSION).cpp)
WRAPPER_OBJECT  = $(WRAPPER:.cpp=.o)
PYOUT           = $(addprefix $(WEBOTS_HOME_PATH)/lib/python$(PYTHON_SHORT_VERSION)/,$(INTERFACE:.i=.py))
PYTHON_PATH_SETUP := $(shell mkdir -p $(WEBOTS_HOME_PATH)/lib/python$(PYTHON_SHORT_VERSION))

ifeq ($(OSTYPE),windows)
SPACE          :=
SPACE          +=
PYTHON_HOME    := $(subst \ ,$(SPACE),$(dir $(subst $(SPACE),\ ,$(shell which python 2> /dev/null))))
C_FLAGS         = -c -O -Wall -DMS_WIN64 -D_hypot=hypot -Wno-stringop-truncation
LD_FLAGS        = -shared -Wl,--enable-auto-import
LIBS            = -L"$(PYTHON_HOME)libs" -lpython$(PYTHON_SHORT_VERSION) -L$(WEBOTS_HOME_PATH)/msys64/mingw64/bin -lController -lCppController
LIBOUT          = $(addprefix $(WEBOTS_HOME_PATH)/lib/python$(PYTHON_SHORT_VERSION)/_,$(INTERFACE:.i=.pyd))
ifneq (,$(findstring 3.,$(PYTHON_VERSION)))
DEF             = _controller3.def
else
DEF             = _controller2.def
endif
PYTHON_INCLUDES = -I"$(PYTHON_HOME)include"
LIBCONTROLLER   = $(WEBOTS_HOME_PATH)/msys64/mingw64/bin/Controller.dll
LIBCPPCONTROLLER= $(WEBOTS_HOME_PATH)/msys64/mingw64/bin/CppController.dll
endif

ifeq ($(OSTYPE),darwin)
ifeq ($(PYTHON_SHORT_VERSION), 27)
PYTHON_PATH ?= /System/Library/Frameworks/Python.framework/Versions/$(PYTHON_VERSION)
PYTHON_PYMALLOC =
else
PYTHON_PATH ?= /Library/Frameworks/Python.framework/Versions/$(PYTHON_VERSION)
PYTHON_PYMALLOC = m
endif
PYTHON_BIN      = $(PYTHON_PATH)/bin/
C_FLAGS         = -c -Wall -fPIC -mmacosx-version-min=$(MACOSX_MIN_SDK_VERSION)
ifeq ($(findstring llvm-g++,$(shell ls -lF $(shell which c++ 2> /dev/null))),)
C_FLAGS        += -Wno-self-assign
endif
LD_FLAGS        = -dynamiclib -install_name @rpath/lib/python$(PYTHON_SHORT_VERSION)/_$(INTERFACE:.i=.dylib) -Wl,-rpath,@loader_path/../.. -compatibility_version 1.0 -current_version 1.0.0 -mmacosx-version-min=$(MACOSX_MIN_SDK_VERSION)
LIBS            = -L"$(PYTHON_PATH)/lib" -L$(WEBOTS_HOME_PATH)/lib -lController -lCppController -lpython$(PYTHON_VERSION)
LIBOUT          = $(addprefix $(WEBOTS_HOME_PATH)/lib/python$(PYTHON_SHORT_VERSION)/_,$(INTERFACE:.i=.so))
PYTHON_INCLUDES = -I"$(PYTHON_PATH)/include/python$(PYTHON_VERSION)$(PYTHON_PYMALLOC)"
LIBCONTROLLER   = $(WEBOTS_HOME_PATH)/lib/libController.dylib
LIBCPPCONTROLLER= $(WEBOTS_HOME_PATH)/lib/libCppController.dylib
endif

ifeq ($(OSTYPE),linux)
C_FLAGS         = -c -Wall -fPIC -Wno-unused-but-set-variable
LD_FLAGS        = -shared
LIBS            = -L$(WEBOTS_HOME_PATH)/lib -lController -lCppController
LIBOUT          = $(addprefix $(WEBOTS_HOME_PATH)/lib/python$(PYTHON_SHORT_VERSION)/_,$(INTERFACE:.i=.so))
PYTHON_INCLUDES = -I"/usr/include/python$(PYTHON_VERSION)"
LIBCONTROLLER   = $(WEBOTS_HOME_PATH)/lib/libController.so
LIBCPPCONTROLLER= $(WEBOTS_HOME_PATH)/lib/libCppController.so
endif

SWIG_EXISTS     = $(shell which $(SWIG) 2> /dev/null)

TARGET          = $(PYOUT) $(LIBOUT)

ifeq (, $(shell which $(PYTHON_COMMAND) 2> /dev/null))
release debug profile:
	@echo -e "# \033[0;33m$(PYTHON_COMMAND) not installed, skipping Python API\033[0m"
else ifeq ($(SWIG_EXISTS),)
release debug profile:
	@echo -e "# \033[0;33mSWIG not installed, skipping Python API\033[0m"
else
release debug profile: $(TARGET)

$(PYOUT) $(WRAPPER):$(INTERFACE) $(LIBCONTROLLER) $(LIBCPPCONTROLLER)
	$(SWIG) $(SWIG_OPTS) $(WEBOTS_INCLUDES) -o $(WRAPPER) $<

$(LIBOUT):$(WRAPPER_OBJECT) $(LIBCONTROLLER) $(LIBCPPCONTROLLER)
	$(CXX) $(LD_FLAGS) $< $(DEF) $(LIBS) -o "$@"

$(LIBCONTROLLER):
	@echo "$(LIBCONTROLLER) doesn't exist"

$(LIBCPPCONTROLLER):
	@echo "$(LIBCPPCONTROLLER) doesn't exist"

$(WRAPPER_OBJECT):$(WRAPPER)
	$(CXX) $(C_FLAGS) $(WEBOTS_INCLUDES) $(PYTHON_INCLUDES) $< -o $@
endif

clean:
	rm -rf *.o *.cpp $(WEBOTS_HOME_PATH)/lib/python*
