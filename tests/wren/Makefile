# Copyright 1996-2024 Cyberbotics Ltd.
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

# CPU tests for WREN caches. Linux/GNU ld section collection omits unused GPU entry points.
WEBOTS_HOME ?= ../..
WREN = $(WEBOTS_HOME)/src/wren
BUILD_DIR ?= build

CPPFLAGS += -DGLM_ENABLE_EXPERIMENTAL -I$(WREN) -I$(WEBOTS_HOME)/include -I$(WEBOTS_HOME)/src/glm
CXXFLAGS += -std=c++11 -O2 -ffunction-sections -fdata-sections
LDFLAGS += -Wl,--gc-sections
SOURCES = camera_frustum.cpp $(addprefix $(WREN)/,Camera.cpp Constants.cpp Frustum.cpp Primitive.cpp Node.cpp TransformNode.cpp Transform.cpp)
HEADERS = $(wildcard $(WREN)/*.hpp) $(wildcard $(WEBOTS_HOME)/include/wren/*.h)

.PHONY: test

test: $(BUILD_DIR)/camera_frustum
	./$(BUILD_DIR)/camera_frustum

$(BUILD_DIR)/camera_frustum: $(SOURCES) $(HEADERS) Makefile
	mkdir -p $(BUILD_DIR)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(SOURCES) $(LDFLAGS) $(LDLIBS) -o $@
