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

TARGETS= \
  weight_formula.pdf \
  weight_formula.pdf.png \
  light_intensity.pdf \
  light_intensity.pdf.png \
  light_attenuation.pdf \
  light_attenuation.pdf.png \
  gauss_markov.pdf \
  gauss_markov.pdf.png \
  direct_light.pdf \
  direct_light.pdf.png \
  radar_received_power.pdf \
  radar_received_power.pdf.png \
  spot_light.pdf \
  spot_light.pdf.png \
  display_opacity.pdf \
  display_opacity.pdf.png\
  texture_transform.pdf \
  texture_transform.pdf.png

all: $(TARGETS)

%.pdf: %.eps
	epstopdf --outfile=temp.pdf $<
	pdfcrop temp.pdf $@

%.pdf.png: %.eps
	convert -density 150x150% $< $@

%.eps: %.dvi
	dvips -Ppdf -E -o $@ $< 2> $@.log

%.dvi: %.tex
	latex $< > /dev/null

clean:
	rm -f *.dvi *.aux *.pdf.png *.pdf *.log *~
