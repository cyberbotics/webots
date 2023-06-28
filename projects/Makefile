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

ifeq ($(WEBOTS_HOME),)
WEBOTS_HOME_ARG=WEBOTS_HOME=$(shell cd .. ; pwd)
endif

release profile debug clean:
	@echo "#"; echo "# !!! languages !!!"
	@+make -s -C languages $@ $(WEBOTS_HOME_ARG)
	@echo "#"; echo "# !!! default !!!"
	@+make -s -C default $@ $(WEBOTS_HOME_ARG)
	@echo "#"; echo "# !!! objects !!!"
	@+make -s -C objects $@ $(WEBOTS_HOME_ARG)
	@echo "#"; echo "# !!! robots !!!"
	@+make -s -C robots $@ $(WEBOTS_HOME_ARG)
	@echo "#"; echo "# !!! samples !!!"
	@+make -s -C samples $@ $(WEBOTS_HOME_ARG)
	@echo "#"; echo "# !!! vehicles !!!"
	@+make -s -C vehicles $@ $(WEBOTS_HOME_ARG)
	@echo "#"; echo "# !!! humans !!!"
	@+make -s -C humans $@ $(WEBOTS_HOME_ARG)

cleanse: clean
