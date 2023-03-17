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

TARGETS = advanced_odometry.Makefile \
					advanced_particle_swarm_optimization.Makefile \
					advanced_particle_swarm_optimization_supervisor.Makefile \
					advanced_genetic_algorithm.Makefile \
					advanced_genetic_algorithm_supervisor.Makefile \
					advanced_path_planning_NF1.Makefile \
					advanced_path_planning_potential_field.Makefile \
					advanced_pattern_recognition.Makefile \
					advanced_pattern_recognition_supervisor.Makefile \
					advanced_slam_1.Makefile \
					advanced_slam_2.Makefile \
					ball.Makefile \
					beginner_robot_controller.Makefile \
					intermediate_behavior_based.Makefile \
					intermediate_finite_state_machine.Makefile \
					intermediate_lawn_mower.Makefile \
					intermediate_lfm.Makefile \
					intermediate_oam.Makefile \
					novice_accelerometer.Makefile \
					novice_camera.Makefile \
					novice_ir_sensors.Makefile \
					novice_k2000.Makefile \
					novice_linear_camera.Makefile \
					novice_motors.Makefile

.PHONY: release debug profile clean

release debug profile clean: $(TARGETS)

%.Makefile:
	+@echo "# make" $(MAKECMDGOALS) $(@:.Makefile=)
	+@make -s -C $(@:.Makefile=) $(MAKECMDGOALS)
