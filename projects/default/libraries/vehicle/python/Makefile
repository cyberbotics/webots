# Copyright 1996-2022 Cyberbotics Ltd.
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

null :=
space := $(null) $(null)
WEBOTS_HOME_PATH?=$(subst $(space),\ ,$(strip $(subst \,/,$(WEBOTS_HOME))))
WEBOTS_HOME_PATH_NO_ESCAPE=$(subst \,/,$(WEBOTS_HOME))
include $(WEBOTS_HOME_PATH)/resources/Makefile.os.include

ifeq ($(OSTYPE),windows)
PYTHON_COMMAND ?= python
else
PYTHON_COMMAND ?= python3
endif
PYTHON_VERSION := $(shell $(PYTHON_COMMAND) --version 2>&1 | sed 's/.* \([0-9]\).\([0-9]*\).*/\1.\2/')
PYTHON_SHORT_VERSION = $(subst .,,$(PYTHON_VERSION))
PYTHON_FULL_VERSION := $(subst Python ,,$(shell $(PYTHON_COMMAND) --version 2>&1))
PYTHON_LIB_FOLDER = $(WEBOTS_CONTROLLER_LIB_PATH)/python$(PYTHON_SHORT_VERSION)
WEBOTS_LIBRARY = 1
MODULE_NAME = vehicle

INTERFACE          = $(MODULE_NAME).i
SOURCE_HEADERS     = $(substr $(WEBOTS_HOME_PATH_NO_ESCAPE),$(WEBOTS_HOME_PATH),$(wildcard $(WEBOTS_HOME_PATH)/include/controller/cpp/webots/*.hpp))
SWIG_OPTS          = -c++ -python -outdir $(PYTHON_LIB_FOLDER) -I$(WEBOTS_HOME_PATH)/src/controller/python/
WRAPPER            = $(MODULE_NAME)$(PYTHON_SHORT_VERSION).cpp
WRAPPER_OBJECT     = $(WRAPPER:.cpp=.o)
PYOUT              = $(PYTHON_LIB_FOLDER)/$(MODULE_NAME).py
PYTHON_VERSION    ?= 2.7
INCLUDES           = -I$(WEBOTS_HOME_PATH)/include/controller/c -I$(WEBOTS_HOME_PATH)/include/controller/cpp
LIB                = -lCppCar -lCppDriver

ifeq ($(OSTYPE),linux)
C_FLAGS         = -c -Wall -fPIC -Wno-unused-but-set-variable
LD_FLAGS        = -shared -Wl,-rpath,'$$ORIGIN'/../
LIBRARY         = $(PYTHON_LIB_FOLDER)/_$(MODULE_NAME).so
PYTHON_INCLUDES = -I"/usr/include/python$(PYTHON_VERSION)"
LIB            += -L"$(WEBOTS_CONTROLLER_LIB_PATH)" -lCppController
endif

ifeq ($(OSTYPE),windows)
PYTHON_HOME    := $(subst \ ,$(space),$(dir $(subst $(space),\ ,$(shell which python 2> /dev/null))))
C_FLAGS         = -c -O -Wall -DMS_WIN64 -D_hypot=hypot -Wno-stringop-truncation
LD_FLAGS        = -shared -Wl,--enable-auto-import
LIBRARY         = $(PYTHON_LIB_FOLDER)/_$(MODULE_NAME).pyd
PYTHON_INCLUDES = -I"$(PYTHON_HOME)include"
LIB            += -L"$(WEBOTS_CONTROLLER_LIB_PATH)" -lCppController -L"$(PYTHON_HOME)libs" -lpython$(PYTHON_SHORT_VERSION)
endif

ifeq ($(OSTYPE),darwin)
PYTHON_PATH          ?= /Library/Frameworks/Python.framework/Versions/$(PYTHON_VERSION)
WRAPPER_OBJECT_X86_64 = $(WRAPPER:.cpp=-x86_64.o)
WRAPPER_OBJECT_ARM64  = $(WRAPPER:.cpp=-arm64.o)
SKIP_PYTHON_PYMALLOC := 38 39 310
ifeq ($(filter $(PYTHON_SHORT_VERSION),$(SKIP_PYTHON_PYMALLOC)),)
PYTHON_PYMALLOC = m
endif
BREW_PYTHON_PATH = /usr/local/Cellar/python/$(PYTHON_FULL_VERSION)/Frameworks/Python.framework/Versions/$(PYTHON_VERSION)
C_FLAGS          = -c -Wall -fPIC -mmacosx-version-min=$(MACOSX_MIN_SDK_VERSION) -stdlib=libc++
ifeq ($(PYTHON_SHORT_VERSION),38)
C_FLAGS          += -Wno-deprecated-declarations
endif
ifeq ($(findstring llvm-g++,$(shell ls -lF $(shell which c++ 2> /dev/null))),)
C_FLAGS         += -Wno-self-assign
endif
LD_FLAGS         = -dynamiclib -install_name @rpath/Contents/lib/controller/python$(PYTHON_SHORT_VERSION)/_$(INTERFACE:.i=.so) -Wl,-rpath,@loader_path/../../.. -compatibility_version 1.0 -current_version 1.0.0 -mmacosx-version-min=$(MACOSX_MIN_SDK_VERSION) -stdlib=libc++
PYTHON_INCLUDES  = -isystem $(PYTHON_PATH)/include/python$(PYTHON_VERSION)$(PYTHON_PYMALLOC)
LIB             += -L"$(WEBOTS_CONTROLLER_LIB_PATH)" -lCppController -L"$(PYTHON_PATH)/lib" -lpython$(PYTHON_VERSION)
LIBRARY          = $(PYTHON_LIB_FOLDER)/_$(MODULE_NAME).so
ifneq (,$(wildcard $(BREW_PYTHON_PATH)))
PYTHON_INCLUDES += -I"$(BREW_PYTHON_PATH)/include/python$(PYTHON_VERSION)$(PYTHON_PYMALLOC)"
LIB             += -L"$(BREW_PYTHON_PATH)/lib"
endif
endif

TARGET = $(PYOUT) $(LIBRARY)

SWIG_EXISTS = $(shell which swig 2> /dev/null)

ifeq (, $(shell which $(PYTHON_COMMAND)-config 2> /dev/null))
release debug profile:
	@echo "# \033[0;33m$(PYTHON_COMMAND)-dev not installed, skipping Python API\033[0m"
else
ifeq ($(SWIG_EXISTS),)
release debug profile:
	@echo "# \033[0;33mSWIG not installed, skipping Python API\033[0m"
else
release debug profile: $(TARGET)

$(PYOUT) $(WRAPPER):$(INTERFACE) $(SOURCE_HEADERS)
	swig $(SWIG_OPTS) $(INCLUDES) -o $(WRAPPER) $<

ifeq ($(OSTYPE),darwin)
$(WRAPPER_OBJECT_ARM64):$(WRAPPER)
	$(CXX) -target arm64-apple-macos11 $(C_FLAGS) $(INCLUDES) $(PYTHON_INCLUDES) $< -o $@

$(WRAPPER_OBJECT_X86_64):$(WRAPPER)
	$(CXX) -target x86_64-apple-macos11 $(C_FLAGS) $(INCLUDES) $(PYTHON_INCLUDES) $< -o $@

ifeq ($(PYTHON_SHORT_VERSION),37)
$(LIBRARY):$(WRAPPER_OBJECT_X86_64)
	$(CXX) -target x86_64-apple-macos11 $(LD_FLAGS) $(WRAPPER_OBJECT_X86_64) $(DEF) $(LIB) -o "$@"
else
$(LIBRARY):$(WRAPPER_OBJECT_X86_64) $(WRAPPER_OBJECT_ARM64)
	$(CXX) -target arm64-apple-macos11 $(LD_FLAGS) $(WRAPPER_OBJECT_ARM64) $(DEF) $(LIB) -o _vehicle$(PYTHON_SHORT_VERSION)-arm64.so
	$(CXX) -target x86_64-apple-macos11 $(LD_FLAGS) $(WRAPPER_OBJECT_X86_64) $(DEF) $(LIB) -o _vehicle$(PYTHON_SHORT_VERSION)-x86_64.so
	lipo -create -output "$@" _vehicle$(PYTHON_SHORT_VERSION)-arm64.so _vehicle$(PYTHON_SHORT_VERSION)-x86_64.so
	codesign -s - "$@"
endif

else
$(WRAPPER_OBJECT):$(WRAPPER)
	$(CXX) $(C_FLAGS) $(INCLUDES) $(PYTHON_INCLUDES) $< -o $@

$(LIBRARY):$(WRAPPER_OBJECT)
	$(CXX) $(LD_FLAGS) $< $(DEF) $(LIB) -o "$@"

endif
endif
endif

clean:
	@rm -fr *.o *.cpp $(PYOUT) $(LIBRARY) $(MODULE_NAME).pyc _*.so
