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

# clear out all suffixes for implicit rules to speed up Makefile
.SUFFIXES:
%:: %,v
%:: RCS/%,v
%:: RCS/%
%:: s.%
%:: SCCS/s.%

ifndef WEBOTS_HOME
export WEBOTS_HOME=../..
else
WEBOTS_HOME := $(WEBOTS_HOME)
endif

null :=
space := $(null) $(null)
WEBOTS_HOME_PATH?=$(subst $(space),\ ,$(strip $(subst \,/,$(WEBOTS_HOME))))

include $(WEBOTS_HOME_PATH)/resources/Makefile.os.include

.DEFAULT_GOAL := help

WEBOTS_SRC_PATH = $(WEBOTS_HOME_PATH)/src/webots

ifeq ($(OSTYPE),windows)
LUPDATE = lupdate-qt6
LRELEASE = lrelease-qt6
else
QT_BINARIES_PATH = $(WEBOTS_HOME_PATH)/bin/qt

ifeq ($(OSTYPE),linux)
LUPDATE = LD_LIBRARY_PATH=$(WEBOTS_LIB_PATH) $(QT_BINARIES_PATH)/lupdate
LRELEASE = LD_LIBRARY_PATH=$(WEBOTS_LIB_PATH) $(QT_BINARIES_PATH)/lrelease
endif

ifeq ($(OSTYPE),darwin)
LUPDATE = DYLD_FRAMEWORK_PATH=$(WEBOTS_HOME)/Contents/Frameworks $(QT_BINARIES_PATH)/lupdate
LRELEASE = DYLD_FRAMEWORK_PATH=$(WEBOTS_HOME)/Contents/Frameworks $(QT_BINARIES_PATH)/lrelease
endif
endif

LUPDATE_FLAGS = -silent -locations none -noobsolete -extensions cpp,hpp
LRELEASE_FLAGS = -silent

GENERIC = wb_generic.ts
TS_SOURCES = $(wildcard *.ts)
QM_FILES = $(subst $(GENERIC:.ts=.qm),,$(TS_SOURCES:.ts=.qm))
ifneq ($(MAKECMDGOALS),update)
WEBOTS_SOURCES =
else
WEBOTS_SOURCES = $(wildcard $(WEBOTS_SRC_PATH)/*/*.[ch]pp)
endif

help:
	@echo Usage: make \<TARGET\>
	@echo \'make release\': rebuild the QM files from TS files.
	@echo \'make update\': \ rebuild the TS files from the Webots sources files.
	@echo \'make clean\': \ \ remove all QM files.

release debug profile update: $(QM_FILES) $(GENERIC)

%.qm: %.ts
	@echo "# compiling $< -> $@"
	@$(LRELEASE) $(LRELEASE_FLAGS) $<

wb_*.ts: $(WEBOTS_SOURCES)
	@echo "# updating $@"
	@$(LUPDATE) $(LUPDATE_FLAGS) $(WEBOTS_SRC_PATH) -ts $@

clean:
	rm -f *.qm
