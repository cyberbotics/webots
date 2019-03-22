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

LIBRARIES=robotis/darwin-op/libraries.Makefile \
	neuronics/ipr/libraries.Makefile \
	mobsya/thymio/libraries.Makefile \
	kuka/youbot/libraries.Makefile

CONTROLLERS=\
  epfl/lis/controllers.Makefile \
  sony/aibo/controllers.Makefile \
  a4/portal/controllers.Makefile \
  adept/pioneer3/controllers.Makefile \
  softbank/nao/controllers.Makefile \
  bluebotics/shrimp/controllers.Makefile \
  boston_dynamics/atlas/controllers.Makefile \
  clearpath/pr2/controllers.Makefile \
  epfl/biorob/controllers.Makefile \
  fujitsu/hoap2/controllers.Makefile \
  gctronic/e-puck/controllers.Makefile \
  gctronic/elisa/controllers.Makefile \
  irobot/create/controllers.Makefile \
  k-team/hemisson/controllers.Makefile \
  k-team/khepera1/controllers.Makefile \
  k-team/khepera3/controllers.Makefile \
  k-team/khepera4/controllers.Makefile \
  kondo/khr-2hv/controllers.Makefile \
  kuka/youbot/controllers.Makefile \
  lego/mindstorms/controllers.Makefile \
  micromagic/mantis/controllers.Makefile \
  mobsya/thymio/controllers.Makefile \
  nasa/controllers.Makefile \
  neuronics/ipr/controllers.Makefile \
  nex/controllers.Makefile \
  parallax/boebot/controllers.Makefile \
  picaxe/microbot/controllers.Makefile \
  robotis/bioloid/controllers.Makefile \
  robotis/darwin-op/controllers.Makefile \
  sony/qrio/controllers.Makefile \
  sphero/controllers.Makefile \
  surveyor/controllers.Makefile \
  tinkerbots/controllers.Makefile \
  unimation/puma/controllers.Makefile

ROBOT_WINDOWS=gctronic/e-puck/plugins/robot_windows.Makefile \
	robotis/darwin-op/plugins/robot_windows.Makefile \
	nex/plugins/robot_windows.Makefile

REMOTE_CONTROLS=gctronic/e-puck/plugins/remote_controls.Makefile \
	robotis/darwin-op/plugins/remote_controls.Makefile \
	nex/plugins/remote_controls.Makefile

PHYSICS_PLUGINS=epfl/lis/plugins/physics.Makefile

RESOURCES=gctronic/e-puck/transfer.Makefile

TARGETS = $(LIBRARIES) $(CONTROLLERS) $(ROBOT_WINDOWS) $(REMOTE_CONTROLS) $(PHYSICS_PLUGINS) $(RESOURCES)

.PHONY: release debug profile clean

release debug profile clean: $(TARGETS)

# libraries dependencies

robotis/darwin-op/controllers.Makefile: robotis/darwin-op/libraries.Makefile

neuronics/ipr/controllers.Makefile: neuronics/ipr/libraries.Makefile

mobsya/thymio/controllers.Makefile: mobsya/thymio/libraries.Makefile

kuka/youbot/controllers.Makefile: kuka/youbot/libraries.Makefile

%.Makefile:
	+@echo "# make" $(MAKECMDGOALS) $(@:.Makefile=)
	+@make -s -C $(@:.Makefile=) $(MAKECMDGOALS)
