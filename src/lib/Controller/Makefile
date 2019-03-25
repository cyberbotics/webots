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

WEBOTS_PATH = ../../..

include $(WEBOTS_PATH)/resources/Makefile.os.include

WEBOTS_DEPENDENCY_PATH ?= $(WEBOTS_PATH)/dependencies

ifeq ($(OSTYPE),darwin)
CFLAGS      = -mmacosx-version-min=$(MACOSX_MIN_SDK_VERSION) -fpascal-strings -Wno-deprecated-declarations -I/Developer/Headers/FlatCarbon -Wall -fno-common
INCLUDE     = -I$(WEBOTS_PATH)/include/controller/c -I$(WEBOTS_PATH)/include -I$(WEBOTS_DEPENDENCY_PATH)/libpng-1.0.12 -I$(WEBOTS_PATH)/include/libjpeg -I$(WEBOTS_DEPENDENCY_PATH)/libtiff/libtiff
LD_FLAGS    = -mmacosx-version-min=$(MACOSX_MIN_SDK_VERSION) -dynamiclib -install_name @rpath/lib/libController.dylib -compatibility_version 1.0 -current_version 1.0.0
STATIC_LIBS = $(WEBOTS_DEPENDENCY_PATH)/libpng-1.0.12/libpng.a $(WEBOTS_DEPENDENCY_PATH)/jpeg-9b/libjpeg.a $(WEBOTS_DEPENDENCY_PATH)/libtiff/libtiff.a
SHARED_LIBS = -lz
TARGET      = $(WEBOTS_PATH)/lib/libController.dylib
CC          = gcc
endif

ifeq ($(OSTYPE),linux)
CFLAGS      = -fPIC -Wall
INCLUDE     = -I$(WEBOTS_PATH)/include/controller/c -I$(WEBOTS_PATH)/include
LD_FLAGS    = -shared -Wl,-soname,libController.so
STATIC_LIBS =
ifeq ($(UBUNTU_VERSION), 18.04)
LIB_PNG      = png16
else
LIB_PNG      = png12
endif
SHARED_LIBS = -ltiff -lm -ljpeg -lpthread -l$(LIB_PNG) -ldl -llzma
TARGET      = $(WEBOTS_PATH)/lib/libController.so
endif

ifeq ($(OSTYPE),windows)
CFLAGS      = -Wall -mwindows
INCLUDE     = -I$(WEBOTS_PATH)/include/controller/c -I$(WEBOTS_PATH)/include -I/mingw64/include
LD_FLAGS    = -shared -mwindows --def Controller.def
STATIC_LIBS = -static -lpng -lz -ljpeg -ltiff -llzma -lzstd
PLATFORM    = win32
CC          = gcc
TARGET      = $(WEBOTS_PATH)/msys64/mingw64/bin/Controller.dll $(WEBOTS_PATH)/msys64/mingw64/lib/Controller.lib
ifeq (,$(wildcard /mingw32/bin/gcc))
NO_GCC32 = 1
else
TARGET     += $(WEBOTS_PATH)/msys64/mingw32/bin/Controller.dll $(WEBOTS_PATH)/msys64/mingw32/lib/Controller.lib
endif
endif

ifeq ($(MAKECMDGOALS),debug)
CFLAGS += -g
endif

ifeq ($(TREAT_WARNINGS_AS_ERRORS),1)
CFLAGS += -Werror
endif

ifeq ($(MAKECMDGOALS),profile)
CFLAGS += -pg -O3 -DNDEBUG
LD_FLAGS += -pg
endif

OBJDIR = build/$(MAKECMDGOALS)
ifneq (,$(findstring $(MAKECMDGOALS),debug profile release))
BUILD_PATH_SETUP := $(shell mkdir -p $(OBJDIR))
ifeq ($(OSTYPE),windows)
BUILD_PATH_SETUP := $(shell mkdir -p $(OBJDIR)/mingw32)
endif
endif

ifdef EXTCONTROLLER
CFLAGS += -DEXTCONTROLLER
endif

ifeq ($(MAKECMDGOALS),release)
CFLAGS   += -O3 -DNDEBUG
ifneq ($(OSTYPE),darwin)
LD_FLAGS += -s
endif
endif

vpath %.o $(OBJDIR)
vpath %.c api util

SOURCES=$(shell ls api/*.c) $(shell ls util/*.c)

OBJECTS=$(notdir $(SOURCES:.c=.o))

# clear out all suffixes for implicit rules to speed up Makefile
.SUFFIXES:
%:: %,v
%:: RCS/%,v
%:: RCS/%
%:: s.%
%:: SCCS/s.%

default:
	@echo "Type one of the following (OSTYPE is \""$(OSTYPE)"\"):"
	@echo "  make debug                 for debug (with gdb symbols)"
	@echo "  make profile               for profiling (with gprof information)"
	@echo "  make release               for finale release"
	@echo "  make clean                 remove build directory"
	@echo "  make count-lines           count the lines of code of libController"

ifneq (,$(filter $(MAKECMDGOALS),debug release profile)) # if $(MAKECMDGOALS) is either of those
-include $(addprefix $(OBJDIR)/,$(notdir $(SOURCES:.c=.d)))
endif

debug profile release: $(TARGET)

ifeq ($(OSTYPE),windows)

MINGW64_OBJECTS = $(addprefix $(OBJDIR)/, $(OBJECTS))
MINGW32_OBJECTS = $(addprefix $(OBJDIR)/mingw32/, $(OBJECTS))

$(WEBOTS_PATH)/msys64/mingw64/bin/Controller.dll: $(MINGW64_OBJECTS) Controller.def
	@echo "# linking Controller.dll and libController.a"
	@$(CC) $(LD_FLAGS) -o $@ $(addprefix $(OBJDIR)/, $(OBJECTS)) -Wl,--out-implib,$(WEBOTS_PATH)/msys64/mingw64/lib/libController.a -L/mingw64/lib $(STATIC_LIBS)
ifeq ($(NO_GCC32),1)
	@echo -e "# \033[0;33mgcc-i686 not installed, skipping 32 bit version of Controller.dll\033[0m"
endif

$(WEBOTS_PATH)/msys64/mingw64/lib/Controller.lib: $(WEBOTS_PATH)/msys64/mingw64/bin/Controller.dll Controller.def
ifeq ($(VISUAL_STUDIO_PATH),)
	@echo -e "# \033[0;33m'VISUAL_STUDIO_PATH' not set, skipping Controller.lib\033[0m"
else
	@echo "# creating Controller.lib"
	@PATH="$(VISUAL_STUDIO_PATH)/VC/bin":"$(VISUAL_STUDIO_PATH)/Common7/IDE":"$(PATH)" lib /machine:X64 /def:Controller.def /out:Controller.lib > /dev/null
	@mv Controller.lib "$(WEBOTS_PATH)/msys64/mingw64/lib/"
	@rm Controller.exp
endif

$(WEBOTS_PATH)/msys64/mingw32/bin/Controller.dll: $(MINGW32_OBJECTS) Controller.def
	@echo "# linking Controller.dll and libController.a (32 bit)"
	@PATH=/mingw32/bin $(CC) -L/mingw32/lib $(LD_FLAGS) -o $@ $(addprefix $(OBJDIR)/mingw32/, $(OBJECTS)) -Wl,--out-implib,$(WEBOTS_PATH)/msys64/mingw32/lib/libController.a $(STATIC_LIBS)

$(WEBOTS_PATH)/msys64/mingw32/lib/Controller.lib: $(WEBOTS_PATH)/msys64/mingw32/bin/Controller.dll Controller.def
ifeq ($(VISUAL_STUDIO_PATH),)
	@echo -e "# \033[0;33m'VISUAL_STUDIO_PATH' not set, skipping Controller.lib (32 bit)\033[0m"
else
	@echo "# creating Controller.lib (32 bit)"
	@PATH="$(VISUAL_STUDIO_PATH)/VC/bin":"$(VISUAL_STUDIO_PATH)/Common7/IDE":"$(PATH)" lib /machine:X86 /def:Controller.def /out:Controller32.lib > /dev/null
	@mv Controller32.lib "$(WEBOTS_PATH)/msys64/mingw32/lib/Controller.lib"
	@rm Controller32.exp
endif

else

$(TARGET): $(OBJECTS)
	@echo "# linking  " $(TARGET)
	@$(CC) $(LD_FLAGS) -o $(TARGET) $(addprefix $(OBJDIR)/, $(OBJECTS)) $(STATIC_LIBS) $(SHARED_LIBS)
	@chmod a-x $(TARGET)
endif

$(OBJDIR)/%.o:%.c
	@echo "# compiling" $<
	@$(CC) -c $(CFLAGS) $(EXTRA_CFLAGS) $(INCLUDE) $< -o $@

$(OBJDIR)/mingw32/%o:%c
	@echo "# compiling" $< "(32 bit)"
	@PATH=/mingw32/bin $(CC) -c $(CFLAGS) $(EXTRA_CFLAGS) $(INCLUDE) $< -o $(OBJDIR)/mingw32/$(notdir $@)

$(OBJDIR)/%.d:%.c
	@echo "# updating " $@
	@echo $(basename $(OBJDIR)/$(notdir $@)).o: $(basename $@).d > $(OBJDIR)/$(notdir $@)
	@echo >> $(OBJDIR)/$(notdir $@)
	@$(CC) $(CFLAGS) -MM -w $(INCLUDE) $< >> $(OBJDIR)/$(notdir $@)

clean:
	@-rm -rf build
	@-rm -fr $(TARGET)
	@-rm -fr "$(WEBOTS_PATH)/msys64/mingw64/lib/libController.a"
	@-rm -fr "$(WEBOTS_PATH)/msys64/mingw32/lib/libController.a"

count-lines:
	wc -l */*.c* */*.h*
