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

# For now, it works only on Linux and Windows

include ../../../resources/Makefile.os.include

ifeq ($(OSTYPE),linux)
GLAD=libglad.a
GLU=GLU
GLUT=glut
OPENGL=GL
WREN=libwren.a
else ifeq ($(OSTYPE),windows)
GLAD=glad.a
GLU=glu32
GLUT=freeglut
OPENGL=opengl32
WREN=wren.a
else
$(error $(OSTYPE) is not supported)
endif

all: default phong

default: default.cpp
	g++ -I../../../include default.cpp ../$(WREN) ../../glad/$(GLAD) -l$(GLUT) -l$(OPENGL) -l$(GLU) -ldl -o default

phong: phong.cpp
	g++ -g -I../../../include phong.cpp ../$(WREN) ../../glad/$(GLAD) -l$(GLUT) -l$(OPENGL) -l$(GLU) -ldl -o phong

clean:
	rm default phong
