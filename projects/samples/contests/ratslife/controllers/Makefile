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

C_TARGETS = contest_manager.Makefile
JAVA_TARGETS=Rat0.Makefile_java Rat1.Makefile_java

.PHONY: release debug profile clean

release debug profile clean: $(C_TARGETS) $(JAVA_TARGETS)

%.Makefile:
	+@echo "# make" $(MAKECMDGOALS) $(@:.Makefile=)
	+@make -s -C $(@:.Makefile=) $(MAKECMDGOALS)

%.Makefile_java:
	+@echo "# make_java" $(MAKECMDGOALS) $(@:.Makefile_java=)
	+@make -s -C $(@:.Makefile_java=) -f Makefile_java $(MAKECMDGOALS)
