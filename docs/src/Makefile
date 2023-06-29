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

.PHONY: dia
.PHONY: math

default:
	@echo "Type one of the following:"
	@echo "  make dia                   for the diagrams"
	@echo "  make math                  for the math formulas"
	@echo "  make clean                 remove useless files"

dia: dia/*.dia
	@echo "Producing diagrams..."
	@make -C dia

math: math/*.tex
	@echo "Producing math formulas..."
	@make -C math

clean:
	@make clean -C math
	@make clean -C dia
