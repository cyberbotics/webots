# Copyright 1996-2025 Cyberbotics Ltd.
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

# Generic Webots build rules.

.SUFFIXES:
MAKEFLAGS += -r

vpath %.d $(BUILD_DIR)
vpath %.c $(SOURCES_DIRECTORIES)
vpath %.cc $(SOURCES_DIRECTORIES)
vpath %.c++ $(SOURCES_DIRECTORIES)
vpath %.cpp $(SOURCES_DIRECTORIES)

.PHONY : clean $(SUPPORTED_TARGETS) jar

$(SUPPORTED_TARGETS): $(TARGETS)

clean: $(EXTRA_TARGETS)

$(BUILD_DIRECTORIES):
	$(SILENT)mkdir -p "$@"

ifdef MAIN_TARGET

MAIN_TARGET_WINDOWS_LIB = $(MAIN_TARGET:.dll=.lib)
ifdef WEBOTS_LIBRARY
MAIN_TARGET_COPY = "$(subst $(space),\ ,$(WEBOTS_CONTROLLER_LIB_PATH))/$(MAIN_TARGET)"
MAIN_TARGET_WINDOWS64_LIB = $(WEBOTS_CONTROLLER_LIB_PATH)/$(MAIN_TARGET_WINDOWS_LIB)
else
MAIN_TARGET_COPY ?= $(MAIN_TARGET)
MAIN_TARGET_WINDOWS64_LIB = $(MAIN_TARGET:.dll=.lib)
endif
ifeq ($(OSTYPE),windows)
DYNAMIC_LINK_FLAGS += -Wl,--out-implib,$(MAIN_TARGET_WINDOWS64_LIB)
FILES_TO_REMOVE += $(MAIN_TARGET_WINDOWS64_LIB)
endif

ifdef BUILD_STATIC_LIBRARY
 LFLAGS += $(STATIC_LINK_FLAGS)
 ifndef VERBOSE
  AR_SUPPRESS_OUTPUT = > /dev/null 2>&1
 endif
else
 LIBRARIES += $(DYNAMIC_LIBRARIES)
 LFLAGS += $(DYNAMIC_LINK_FLAGS) -o
endif

ifdef TARGET_LIB_DIR

$(MAIN_TARGET): $(TARGET_LIB_DIR)/$(MAIN_TARGET)

else

$(MAIN_TARGET): $(BUILD_GOAL_DIR)/$(MAIN_TARGET)
	@# Even when compiled from Webots with possibly the previous version of the binary
	@# being executed, it is safe to delete it and copy the new version as a replacement.
	@# This new version will be executed only when the process restarts.
	@# Note: this binary may be an executable (e.g., a controller) or shared library (e.g., a physics plug-in).
	@rm -f $(MAIN_TARGET) > /dev/null 2>&1 && echo "# copying to" $(MAIN_TARGET_COPY) && cp $(BUILD_GOAL_DIR)/$(MAIN_TARGET) $(MAIN_TARGET_COPY) > /dev/null 2>&1

endif

ifdef FAT_BINARY
X86_64_OBJECTS = $(addprefix $(BUILD_GOAL_DIR)/x86_64/, $(OBJECTS_LIST))
ARM64_OBJECTS = $(addprefix $(BUILD_GOAL_DIR)/arm64/, $(OBJECTS_LIST))

$(BUILD_GOAL_DIR)/$(MAIN_TARGET): $(BUILD_GOAL_DIR)/x86_64/$(MAIN_TARGET) $(BUILD_GOAL_DIR)/arm64/$(MAIN_TARGET) | $(BUILD_GOAL_DIR)
	@echo "# creating fat" $(notdir $@)
	$(SILENT)lipo -create -output $@ $(BUILD_GOAL_DIR)/x86_64/$(MAIN_TARGET) $(BUILD_GOAL_DIR)/arm64/$(MAIN_TARGET)

$(TARGET_LIB_DIR)/$(MAIN_TARGET): $(BUILD_GOAL_DIR)/x86_64/$(MAIN_TARGET) $(BUILD_GOAL_DIR)/arm64/$(MAIN_TARGET)
	@echo "# creating fat" $(notdir $@)
	$(SILENT)lipo -create -output $@ $(BUILD_GOAL_DIR)/x86_64/$(MAIN_TARGET) $(BUILD_GOAL_DIR)/arm64/$(MAIN_TARGET)

$(BUILD_GOAL_DIR)/x86_64/$(MAIN_TARGET): $(X86_64_OBJECTS) $(LINK_DEPENDENCIES) | $(BUILD_GOAL_DIR)/x86_64
	@echo "# linking" $(notdir $@) "(x86_64)"
	$(SILENT)$(LINKER) $(X86_64_LINKER) $(LFLAGS) $@ $(X86_64_OBJECTS) $(LIBRARIES) $(AR_SUPPRESS_OUTPUT)

$(BUILD_GOAL_DIR)/arm64/$(MAIN_TARGET): $(ARM64_OBJECTS) $(LINK_DEPENDENCIES) | $(BUILD_GOAL_DIR)/arm64
	@echo "# linking" $(notdir $@) "(arm64)"
	$(SILENT)$(LINKER) $(ARM64_LINKER) $(LFLAGS) $@ $(ARM64_OBJECTS) $(LIBRARIES) $(AR_SUPPRESS_OUTPUT)

else

OBJECTS = $(addprefix $(BUILD_GOAL_DIR)/, $(OBJECTS_LIST))
$(BUILD_GOAL_DIR)/$(MAIN_TARGET): $(OBJECTS) $(LINK_DEPENDENCIES) | $(BUILD_GOAL_DIR)
	@echo "# linking" $(notdir $@)
	$(SILENT)$(LINKER) $(LFLAGS) $@ $(OBJECTS) $(LIBRARIES) $(AR_SUPPRESS_OUTPUT)

$(TARGET_LIB_DIR)/$(MAIN_TARGET): $(OBJECTS) $(LINK_DEPENDENCIES)
	@echo "# linking" $(notdir $@)
	$(SILENT)$(LINKER) $(LFLAGS) $@ $(OBJECTS) $(LIBRARIES) $(AR_SUPPRESS_OUTPUT)

endif

endif

ifdef WREN
$(WREN_TARGET): $(OBJECTS_EMCC)
	$(SILENT) emcc -O3 -s WASM=1 -s EXIT_RUNTIME=1 -s USE_WEBGL2=1 -s FULL_ES3=1 -s ALLOW_MEMORY_GROWTH=1 --preload-file ../../resources/wren/shaders@../../resources/wren/shaders $(OBJECTS_EMCC) -s EXPORTED_FUNCTIONS='[$(shell cat functions_to_export.txt)]' -s 'EXPORTED_RUNTIME_METHODS=["ccall", "getValue"]' -o wrenjs.js
	$(SILENT) cp wrenjs.js wrenjs.wasm wrenjs.data $(WRENJS_DIR)/ > /dev/null 2>&1
endif

$(BUILD_GOAL_DIR)/%.d: %.c | $(BUILD_GOAL_DIR)
	@echo "# updating" $(notdir $@)
	$(SILENT)$(CC) $(INCLUDE) $(WBCFLAGS) $(CFLAGS) -MM $< -MT $(@:.d=.o) > $@

$(BUILD_GOAL_DIR)/%.d: %.cc | $(BUILD_GOAL_DIR)
	@echo "# updating" $(notdir $@)
	$(SILENT)$(CXX) $(INCLUDE) $(WBCFLAGS) $(WBCXXFLAGS) $(CFLAGS) $(CXXFLAGS) -MM $< -MT $(@:.d=.o) > $@

$(BUILD_GOAL_DIR)/%.d: %.c++ | $(BUILD_GOAL_DIR)
	@echo "# updating" $(notdir $@)
	$(SILENT)$(CXX) $(INCLUDE) $(WBCFLAGS) $(WBCXXFLAGS) $(CFLAGS) $(CXXFLAGS) -MM $< -MT $(@:.d=.o) > $@

$(BUILD_GOAL_DIR)/%.d: %.cpp | $(BUILD_GOAL_DIR)
	@echo "# updating" $(notdir $@)
	$(SILENT)$(CXX) $(INCLUDE) $(WBCFLAGS) $(WBCXXFLAGS) $(CFLAGS) $(CXXFLAGS) -MM $< -MT $(@:.d=.o) > $@

ifdef GOAL
 C_DEPENDENCIES = $(patsubst %.c,$(BUILD_GOAL_DIR)/%.d,$(notdir $(filter %.c,$(C_SOURCES))))
 CXX_DEPENDENCIES = $(patsubst %.cc,$(BUILD_GOAL_DIR)/%.d,$(notdir $(filter %.cc,$(CXX_SOURCES)))) \
                    $(patsubst %.c++,$(BUILD_GOAL_DIR)/%.d,$(notdir $(filter %.c++,$(CXX_SOURCES)))) \
                    $(patsubst %.cpp,$(BUILD_GOAL_DIR)/%.d,$(notdir $(filter %.cpp,$(CXX_SOURCES))))
 DEPENDENCIES = $(C_DEPENDENCIES) $(CXX_DEPENDENCIES)
 ifneq ($(filter clean,$(MAKECMDGOALS)),clean)
  ifneq ($(strip $(DEPENDENCIES)),)
  -include $(DEPENDENCIES)
  endif
 endif
endif

$(BUILD_GOAL_DIR)/%.o: %.c | $(BUILD_GOAL_DIR)
	@echo "# compiling" $(notdir $<)
	$(SILENT)$(CC) -c $(WBCFLAGS) $(CFLAGS) $(INCLUDE) $< -o $@

$(BUILD_GOAL_DIR)/x86_64/%.o: %.c | $(BUILD_GOAL_DIR)/x86_64
	@echo "# compiling" $(notdir $<) "(x86_64)"
	$(SILENT)$(CC) -c -target x86_64-apple-macos12 $(WBCFLAGS) $(CFLAGS) $(INCLUDE) $< -o $@

$(BUILD_GOAL_DIR)/arm64/%.o: %.c | $(BUILD_GOAL_DIR)/arm64
	@echo "# compiling" $(notdir $<) "(arm64)"
	$(SILENT)$(CC) -c -target arm64-apple-macos12 $(WBCFLAGS) $(CFLAGS) $(INCLUDE) $< -o $@

$(BUILD_GOAL_DIR)/%.o: %.cc | $(BUILD_GOAL_DIR)
	@echo "# compiling" $(notdir $<)
	$(SILENT)$(CXX) -c $(WBCFLAGS) $(WBCXXFLAGS) $(CFLAGS) $(CXXFLAGS) $(INCLUDE) $< -o $@

$(BUILD_GOAL_DIR)/%.o: %.c++ | $(BUILD_GOAL_DIR)
	@echo "# compiling" $(notdir $<)
	$(SILENT)$(CXX) -c $(WBCFLAGS) $(WBCXXFLAGS) $(CFLAGS) $(CXXFLAGS) $(INCLUDE) $< -o $@

$(BUILD_GOAL_DIR)/%.o: %.cpp | $(BUILD_GOAL_DIR)
	@echo "# compiling" $(notdir $<)
	$(SILENT)$(CXX) -c $(WBCFLAGS) $(WBCXXFLAGS) $(CFLAGS) $(CXXFLAGS) $(INCLUDE) $< -o $@

$(BUILD_GOAL_DIR)/x86_64/%.o: %.cc | $(BUILD_GOAL_DIR)/x86_64
	@echo "# compiling" $(notdir $<) "(x86_64)"
	$(SILENT)$(CXX) -c -target x86_64-apple-macos12 $(WBCFLAGS) $(WBCXXFLAGS) $(CFLAGS) $(CXXFLAGS) $(INCLUDE) $< -o $@

$(BUILD_GOAL_DIR)/arm64/%.o: %.cc | $(BUILD_GOAL_DIR)/arm64
	@echo "# compiling" $(notdir $<) "(arm64)"
	$(SILENT)$(CXX) -c -target arm64-apple-macos12 $(WBCFLAGS) $(WBCXXFLAGS) $(CFLAGS) $(CXXFLAGS) $(INCLUDE) $< -o $@

$(BUILD_GOAL_DIR)/x86_64/%.o: %.c++ | $(BUILD_GOAL_DIR)/x86_64
	@echo "# compiling" $(notdir $<) "(x86_64)"
	$(SILENT)$(CXX) -c -target x86_64-apple-macos12 $(WBCFLAGS) $(WBCXXFLAGS) $(CFLAGS) $(CXXFLAGS) $(INCLUDE) $< -o $@

$(BUILD_GOAL_DIR)/arm64/%.o: %.c++ | $(BUILD_GOAL_DIR)/arm64
	@echo "# compiling" $(notdir $<) "(arm64)"
	$(SILENT)$(CXX) -c -target arm64-apple-macos12 $(WBCFLAGS) $(WBCXXFLAGS) $(CFLAGS) $(CXXFLAGS) $(INCLUDE) $< -o $@

$(BUILD_GOAL_DIR)/x86_64/%.o: %.cpp | $(BUILD_GOAL_DIR)/x86_64
	@echo "# compiling" $(notdir $<) "(x86_64)"
	$(SILENT)$(CXX) -c -target x86_64-apple-macos12 $(WBCFLAGS) $(WBCXXFLAGS) $(CFLAGS) $(CXXFLAGS) $(INCLUDE) $< -o $@

$(BUILD_GOAL_DIR)/arm64/%.o: %.cpp | $(BUILD_GOAL_DIR)/arm64
	@echo "# compiling" $(notdir $<) "(arm64)"
	$(SILENT)$(CXX) -c -target arm64-apple-macos12 $(WBCFLAGS) $(WBCXXFLAGS) $(CFLAGS) $(CXXFLAGS) $(INCLUDE) $< -o $@

$(BUILD_GOAL_DIR_EMCC)/%.o: %.cpp | $(BUILD_GOAL_DIR_EMCC)
	@echo "# compiling with emcc " $(notdir $<)
	$(SILENT)emcc -O3 -c $(WBCFLAGS) $(WBCXXFLAGS) $(CFLAGS) $(CXXFLAGS) $(INCLUDE) $< -o $(@)

clean:
	$(SILENT)rm -fr $(FILES_TO_REMOVE) $(BUILD_DIR) > /dev/null 2>&1 || :
	$(SILENT)rm -d $(EMPTY_DIRECTORIES_TO_REMOVE) > /dev/null 2>&1 || :
  ifdef WREN
	 $(SILENT)rm $(WEBOTS_HOME_PATH)/src/wren/wrenjs.js $(WEBOTS_HOME_PATH)/src/wren/wrenjs.data $(WEBOTS_HOME_PATH)/src/wren/wrenjs.wasm  $(WEBOTS_HOME_PATH)/src/wren/functions_to_export.txt > /dev/null 2>&1 || :
	 $(SILENT)rm $(WRENJS_DIR)/wrenjs.js $(WRENJS_DIR)/wrenjs.data $(WRENJS_DIR)/wrenjs.wasm > /dev/null 2>&1 || :
  endif
  $(SILENT)rm -r $(WRENJS_DIR)/images/post_processing > /dev/null 2>&1 || :
