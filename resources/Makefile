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

distrib: release

release debug profile:
	@echo "#"; echo "# *** translations ***"
	@+make -s -C translations $@
	@echo "#"; echo "# *** languages ***"
	@+make -s -C languages $@
	@echo "#"; echo "# *** projects ***"
	@+make -s -C projects $@
	@echo "#"; echo "# *** web/wwi ***"
	@+make -s -C web/wwi $@

cleanse: clean

clean:
	@+make -s -C translations $@
	@+make -s -C languages $@
	@+make -s -C projects $@
	@-rm -rf sumo_exporter/*.pyc
	@+make -s -C web/wwi $(MAKECMDGOALS)
