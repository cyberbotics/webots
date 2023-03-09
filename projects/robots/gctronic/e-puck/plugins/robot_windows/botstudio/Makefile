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

# Webots Makefile system
#
# You may add some variable definitions hereafter to customize the build process
# See documentation in $(WEBOTS_HOME_PATH)/resources/Makefile.include


CXX_SOURCES = \
  core/Automaton.cpp \
  core/AutomatonObject.cpp \
  core/AutomatonObjectRepresentation.cpp \
  core/AutomatonScene.cpp \
  core/AutomatonWidget.cpp \
  core/BotStudioWindow.cpp \
  core/BotStudioPaths.cpp \
  core/CommonProperties.cpp \
  core/Model.cpp \
  core/RemoteControlWidget.cpp \
  core/RobotConditionWidget.cpp \
  core/RobotObjectFactory.cpp \
  core/RobotStateWidget.cpp \
  core/RobotViewWidget.cpp \
  core/State.cpp \
  core/StateRepresentation.cpp \
  core/SwitchWidget.cpp \
  core/Tokenizer.cpp \
  core/Transition.cpp \
  core/TransitionRepresentation.cpp \
  core/entry_points.cpp \
  e-puck/EPuckActuatorCommand.cpp \
  e-puck/EPuckConditionWidget.cpp \
  e-puck/EPuckDrawingHelper.cpp \
  e-puck/EPuckFacade.cpp \
  e-puck/EPuckLedButton.cpp \
  e-puck/EPuckObjectFactory.cpp \
  e-puck/EPuckSensorCondition.cpp \
  e-puck/EPuckSlider.cpp \
  e-puck/EPuckStateWidget.cpp \
  e-puck/EPuckViewWidget.cpp

HPP_FILES_TO_MOC = \
  core/Automaton.hpp \
  core/AutomatonObject.hpp \
  core/AutomatonObjectRepresentation.hpp \
  core/AutomatonScene.hpp \
  core/AutomatonWidget.hpp \
  core/BotStudioWindow.hpp \
  core/Model.hpp \
  core/RemoteControlWidget.hpp \
  core/RobotFacade.hpp \
  core/State.hpp \
  core/StateRepresentation.hpp \
  core/SwitchWidget.hpp \
  core/TransitionRepresentation.hpp \
  e-puck/EPuckConditionWidget.hpp \
  e-puck/EPuckLedButton.hpp \
  e-puck/EPuckSlider.hpp \
  e-puck/EPuckStateWidget.hpp
USE_C_API = true

QT_UTILS = $(WEBOTS_HOME)/resources/projects/libraries/qt_utils
INCLUDE = -I"$(QT_UTILS)"
LIBRARIES = -L"$(QT_UTILS)" -lqt_utils

null :=
space := $(null) $(null)
WEBOTS_HOME_PATH?=$(subst $(space),\ ,$(strip $(subst \,/,$(WEBOTS_HOME))))
include $(WEBOTS_HOME_PATH)/resources/Makefile.include

# wb_robot_get_controller_name() is deprecated
WBCFLAGS += -std=c++17 -Wno-deprecated-declarations
MOC_SOURCES = $(addprefix $(BUILD_DIR)/,$(notdir $(HPP_FILES_TO_MOC:.hpp=.moc.cpp)))
MOC_DIRECTORIES = $(sort $(dir $(HPP_FILES_TO_MOC)))

ifeq ($(OSTYPE),windows)
 CFLAGS += -D_USE_MATH_DEFINES
 QT_INCLUDE_DIR = $(WEBOTS_HOME)/include/qt
 MOC = /mingw64/share/qt6/bin/moc
 MOC_PLATFORM_FLAGS = -D_WIN32
 DYNAMIC_LIBRARIES += -L"$(WEBOTS_HOME)/msys64/mingw64/bin" -lQt6Core -lQt6Gui -lQt6Widgets -lQt6Concurrent
 INCLUDE += -isystem "$(QT_INCLUDE_DIR)/QtCore" -isystem "$(QT_INCLUDE_DIR)/QtGui" -isystem "$(QT_INCLUDE_DIR)/QtWidgets" -isystem "$(QT_INCLUDE_DIR)/QtConcurrent"
 OBJECTS += $(addprefix $(BUILD_GOAL_DIR)/,$(notdir $(MOC_SOURCES:.cpp=.o)))
endif

ifeq ($(OSTYPE),linux)
 QT_INCLUDE_DIR = $(WEBOTS_HOME)/include/qt
 QT_LIB_DIR = $(WEBOTS_HOME)/lib/webots
 MOC = "$(WEBOTS_HOME)/bin/qt/moc"
 MOC_PLATFORM_FLAGS = -D__linux__
 DYNAMIC_LIBRARIES += -L"$(QT_LIB_DIR)" -lQt6Core -lQt6Gui -lQt6Widgets -lQt6Concurrent
 INCLUDE += -isystem "$(QT_INCLUDE_DIR)/QtCore" -isystem "$(QT_INCLUDE_DIR)/QtGui" -isystem "$(QT_INCLUDE_DIR)/QtWidgets" -isystem "$(QT_INCLUDE_DIR)/QtConcurrent"
 OBJECTS += $(addprefix $(BUILD_GOAL_DIR)/,$(notdir $(MOC_SOURCES:.cpp=.o)))
endif

ifeq ($(OSTYPE),darwin)
 WBCFLAGS += -Wno-unknown-pragmas
 FRAMEWORKS_DIR = $(WEBOTS_HOME)/Contents/Frameworks
 INCLUDE += -F"$(FRAMEWORKS_DIR)"
 FRAMEWORKS += -F"$(FRAMEWORKS_DIR)"
 DYNAMIC_LIBRARIES += -bind_at_load $(FRAMEWORKS)
 MOC = "$(WEBOTS_HOME)/bin/qt/moc"
 MOC_PLATFORM_FLAGS = -D__APPLE__
 FRAMEWORKS += -framework QtCore -framework QtGui -framework QtWidgets -framework QtConcurrent
 X86_64_OBJECTS += $(addprefix $(BUILD_GOAL_DIR)/x86_64/,$(notdir $(MOC_SOURCES:.cpp=.o)))
 ARM64_OBJECTS += $(addprefix $(BUILD_GOAL_DIR)/arm64/,$(notdir $(MOC_SOURCES:.cpp=.o)))
endif

vpath %.cpp $(BUILD_DIR)
vpath %.hpp $(MOC_DIRECTORIES)

.PRECIOUS: $(MOC_SOURCES)

$(BUILD_DIR)/%.moc.cpp: %.hpp
	@echo "# moc-generating" $(notdir $<)
	$(SILENT)$(MOC) $(MOC_PLATFORM_FLAGS) $(INCLUDE:-isystem:-I) $< -o $@

ifeq ($(OSTYPE),darwin)
$(BUILD_GOAL_DIR)/$(MAIN_TARGET): $(BUILD_GOAL_DIR)/x86_64/$(MAIN_TARGET) $(BUILD_GOAL_DIR)/arm64/$(MAIN_TARGET)
$(BUILD_GOAL_DIR)/x86_64/$(MAIN_TARGET): $(X86_64_OBJECTS)
$(BUILD_GOAL_DIR)/arm64/$(MAIN_TARGET): $(ARM64_OBJECTS)
else
$(BUILD_GOAL_DIR)/$(MAIN_TARGET): $(OBJECTS)
endif
