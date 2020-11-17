#!/usr/bin/env python3

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

import os
from pyclibrary import CParser

listheaders = os.listdir("../../include/wren/")
buggyheaders = {"config.h", "drawable_texture.h", "file_import.h", "font.h", "overlay.h"}
listheaders = ["../../include/wren/" + header for header in listheaders if header[len(header)-2:len(header)] == '.h' and not(header in buggyheaders)]

print(listheaders)
parser = CParser(listheaders)

parser.process_all()

#parser.print_all()

all_values = parser.defs['values']
functionSignatures = parser.defs['functions']
functionName = functionSignatures.keys()
functionName = map(lambda name : "_"+ name + ", ", functionName)

if os.path.exists("../../src/wren/functionsToExport.txt"):
  os.remove("../../src/wren/functionsToExport.txt")

functionName = list(functionName)
lastIndex= len(functionName) - 1
lastName = functionName[lastIndex];
lastName = lastName[:len(lastName)-2]
functionName[lastIndex] = lastName;

f = open("../../src/wren/functionsToExport.txt", 'x')

f.write(''.join(functionName));

