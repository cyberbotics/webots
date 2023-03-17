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

START := $(shell date +%s)

ifeq ($(WEBOTS_HOME),)
ifneq ($(findstring MINGW,$(shell uname)),) # under MINGW, we need to set WEBOTS_HOME using the native Windows format
export WEBOTS_HOME:=`pwd -W | tr -s / '\\'`
else
export WEBOTS_HOME = $(PWD)
endif
endif

include resources/Makefile.os.include

ifeq ($(JAVA_HOME)$(OSTYPE),linux)
JAVAC:=$(shell which javac)
ifneq ($(JAVAC),)
export JAVA_HOME:=$(shell dirname $(dir $(shell readlink -f $(JAVAC))))
endif
endif

WEBOTS_DISTRIBUTION_PATH ?= $(WEBOTS_HOME)/distribution

ifeq ($(MAKECMDGOALS),)
MAKECMDGOALS = release
else
ifeq ($(MAKECMDGOALS),webots_target)
MAKECMDGOALS = release
endif
endif

ifeq ($(MAKECMDGOALS),distrib)
TARGET = release
export TREAT_WARNINGS_AS_ERRORS=1
else
ifeq ($(MAKECMDGOALS),cleanse)
TARGET = clean
else
TARGET = $(MAKECMDGOALS)
endif
endif

null :=
space := $(null) $(null)
WEBOTS_HOME_PATH?=$(subst $(space),\ ,$(strip $(subst \,/,$(WEBOTS_HOME))))
include $(WEBOTS_HOME_PATH)/resources/Makefile.os.include

.PHONY: clean cleanse debug distrib release webots_dependencies webots_target webots_projects clean-docs docs clean-urls

release debug profile: docs webots_projects

distrib: release
	@+echo "#"; echo "# packaging"; echo "#"
	@+make --silent -C scripts/packaging
	$(eval DT := `expr \`date +%s\` - $(START)`)
	@printf "# distribution compiled in %d:%02d:%02d\n" $$(($(DT) / 3600)) $$(($(DT) % 3600 / 60)) $$(($(DT) % 60))

ifeq ($(OSTYPE),windows)
CLEAN_IGNORE += -e lib/webots/qt -e include/qt
endif

# we should make clean before building a release
clean: webots_projects clean-docs clean-urls
	@+echo "#"; echo "# * packaging *"; echo "#"
	@+make --silent -C scripts/packaging clean
	@+echo "#"; echo "# remove OS generated files and text editor backup files"
	@+find . -type f \( -name "*~" -o -name "*.bak" -o -name ".DS_Store" -o -name ".DS_Store?" -o -name ".Spotlight-V100" -o -name ".Trashes" -o -name "__pycache__" -o -name "Thumbs.db" -o -name "ehthumbs.db" \) -exec /bin/rm -f -- {} + -exec echo "# removed" {} +
	@+find . -type d \( -name "__pycache__" \) -exec /bin/rm -rf -- {} + -exec echo "# removed" {} +
ifeq ($(MAKECMDGOALS),clean)
	@+echo "#"; echo "# testing if everything was cleaned..."
	@+git clean -fdfxn -e tests $(CLEAN_IGNORE)
	@+echo "# done"
endif

# cleanse is the ultimate cleansing (agressive cleaning)
cleanse: clean
	@rm -fr docs/index.html docs/dependencies
	@rm -rf $(WEBOTS_DISTRIBUTION_PATH)/*
ifeq ($(OSTYPE),windows)
	@rm -rf msys64
endif
ifeq ($(OSTYPE),darwin)
	@+make --silent -C dependencies -f Makefile.mac $(MAKECMDGOALS)
endif
	@+echo "#"; echo "# * tests *"; echo "#"
	@find tests -name .*.cache | xargs rm -f
	@+make --silent -C tests clean
	@+echo "#"; echo "# testing if everything was cleansed..."
	@+git clean -fdfxn $(CLEAN_IGNORE)
	@+echo "# done"

webots_target: webots_dependencies
	@+echo "#"; echo "# * ode *"; echo "#"
	@+make --silent -C src/ode $(TARGET)
ifeq ($(TARGET),profile)  # a shared version of the library is required for physics-plugins
	@+make --silent -C src/ode release
endif
	@+echo "#"; echo "# * glad *"; echo "#"
	@+make --silent -C src/glad $(TARGET)
	@+echo "#"; echo "# * wren *"; echo "#"
	@+make --silent -C src/wren $(TARGET)
	@+echo "#"; echo "# * webots (core) *"; echo "#"
	@+make --silent -C src/webots $(TARGET)

webots_projects: webots_target
	@+echo "#"; echo "# * controller library *"
	@+make --silent -C src/controller $(TARGET) WEBOTS_HOME="$(WEBOTS_HOME)"
	@+echo "#"; echo "# * resources *"
	@+make --silent -C resources $(MAKECMDGOALS) WEBOTS_HOME="$(WEBOTS_HOME)"
	@+echo "#"; echo "# * projects *"
	@+make --silent -C projects $(TARGET) WEBOTS_HOME="$(WEBOTS_HOME)"

webots_dependencies:
	@+echo "#"; echo "# * dependencies *"; echo "#"
ifeq ($(OSTYPE),darwin)
	@+make --silent -C dependencies -f Makefile.mac $(MAKECMDGOALS)
endif
ifeq ($(OSTYPE),linux)
	@+make --silent -C dependencies -f Makefile.linux $(MAKECMDGOALS)
endif
ifeq ($(OSTYPE),windows)
	@+make --silent -C dependencies -f Makefile.windows $(MAKECMDGOALS)
endif
ifneq ($(TARGET),clean)
	@+python3 scripts/packaging/generate_proto_list.py
else
	@+rm -f resources/proto-list.xml
endif

ifeq ($(OSTYPE),darwin)
NUMBER_OF_PROCESSORS = `sysctl -n hw.ncpu`
else
NUMBER_OF_PROCESSORS ?= `grep -c ^processor /proc/cpuinfo`
endif
THREADS = $$(($(NUMBER_OF_PROCESSORS) * 3 / 2))

docs:
	@$(WEBOTS_HOME_PATH)/scripts/get_git_info/get_git_info.sh
	@$(shell find $(WEBOTS_HOME_PATH)/docs -name '*.md' | sed 's/.*docs[/]//' > $(WEBOTS_HOME_PATH)/docs/list.txt)

clean-docs:
	@+echo "#"; echo "# * documentation *"
	@-rm -f docs/list.txt

clean-urls:
	@+echo "#"; echo "# * clean URLs *"
	@+python3 scripts/packaging/update_urls.py webots

install:
	@+echo "#"; echo "# * installing (snap) *"
	@+make --silent -C scripts/packaging -f Makefile install

help:
	@+echo
	@+$(ECHO) "\033[32;1mWebots Makefile targets:\033[0m"
	@+echo
	@+$(ECHO) "\033[33;1mmake -j$(THREADS) release\033[0m\t# compile with maximum optimization (default)"
	@+$(ECHO) "\033[33;1mmake -j$(THREADS) debug\033[0m  \t# compile with gdb debugging symbols"
	@+$(ECHO) "\033[33;1mmake -j$(THREADS) profile\033[0m\t# compile with gprof profiling information"
	@+$(ECHO) "\033[33;1mmake -j$(THREADS) distrib\033[0m\t# compile in release mode & create distribution package"
	@+$(ECHO) "\033[33;1mmake -j$(THREADS) clean\033[0m  \t# clean-up the compilation output"
	@+$(ECHO) "\033[33;1mmake -j$(THREADS) cleanse\033[0m\t# deep clean-up (dependencies are also removed)"
	@+$(ECHO) "\033[33;1mmake help\033[0m\t\t# display this message and exit"
	@+echo
	@+$(ECHO) "\033[32;1mNote:\033[0m You seem to have a processor with $(NUMBER_OF_PROCESSORS) virtual cores,"
	@+$(ECHO) "      hence the \033[33;1m-j$(THREADS)\033[0m option to speed-up the compilation."
