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

# Convert .dia into both .pdf and .pdf.png
# for Webots documentation
# written by yvan.bourquin@epfl.ch

REF_TARGETS= \
 ray_orbits.pdf \
 ray_orbits.pdf.png \
 hinge_joint.pdf \
 hinge_joint.pdf.png \
 linear_motor.pdf \
 linear_motor.pdf.png \
 rotational_motor.pdf \
 rotational_motor.pdf.png \
 connector_alignment.pdf \
 connector_alignment.pdf.png \
 emitter_receiver.pdf \
 emitter_receiver.pdf.png \
 receiver_queue.pdf \
 receiver_queue.pdf.png \
 slider_joint.pdf \
 slider_joint.pdf.png \
 light_sensor.pdf \
 light_sensor.pdf.png \
 oo_api.pdf \
 oo_api.pdf.png

all : $(REF_TARGETS)

%.pdf : %.eps
	epstopdf --outfile=$@ $<

%.eps : %.dia
	dia -e $@ $<

%.pdf.png : %.eps
	convert $< $@

clean :
	rm -f *.pdf *.eps *.png *.dia~ *.log
