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
#
# Author Simon Steinmann (https://github.com/Simon-Steinmann)


# Automatic PROTO to multi-file PROTO trimesh extractor
#
# INSTRUCTIONS:
# Call the script with the --input=<path> argument
#
# 1. if <path> ends in <filename.proto>:
#   - creates a new folder in the same directory named "multifile_<filename>"
#   - creates a new <filename.proto> inside this folder with all trimeshes replaced
#      by proto files, placed in a subfolder
#
# 2. If <path> does not end in ".proto" -> assumes directory
#   - creates a new folder inside of <path> called <multi_file_conversion>
#   - searches for every .proto file in <path> recursively and:
#       - creates a conversion as described in 1.
#       - places conversion into the <multi_file_conversion> folder, preserving
#         the same folder structure as in <path>. This allows <path> to be replaced
#         with <path>/<multi_file_conversion> when validated, as names structures
#         are preserved
#


import os
import optparse
import errno


def mkdirSafe(directory):
    """Create a dir safely."""
    try:
        os.makedirs(directory)
    except OSError as e:
        if e.errno != errno.EEXIST:
            raise
        else:
            print('Directory "' + directory + '" already exists!')


class proto2multi():
    def __init__(self):
        print('Proto 2 multi-file proto converter by Simon Steinmann')

    def header(self, proto):
        """Specify VRML file header."""
        proto.write('#VRML_SIM R2020b utf8\n')
        proto.write('# license: Apache License 2.0\n')
        proto.write('# license url: http://www.apache.org/licenses/LICENSE-2.0\n')
        proto.write('# tags: hidden\n')
        proto.write('# This is a proto file for Webots for the ' + self.robotName + '\n\n')

    def createProto(self, string):
        name = self.robotName + '_' + str(self.shapeIndex)
        print('Create meshFile: %sMesh.proto' % name)
        filepath = '%s/%sMesh.proto' % (self.meshFilesPath, name)
        meshProtoFile = open(filepath, 'w')
        self.header(meshProtoFile)
        meshProtoFile.write('PROTO %sMesh [\n]\n{\n' % name)
        meshProtoFile.write(string)
        meshProtoFile.write('}\n')
        meshProtoFile.close()
        replaceString = '%sMesh {\n' % name
        self.shapeIndex += 1
        return replaceString

    def convert(self, inFile, outFile=None):
        path = os.path.dirname(inFile)
        self.robotName = os.path.basename(inFile)
        self.robotName = os.path.splitext(self.robotName)[0]
        print(path, self.robotName)
        if outFile is None:
            newPath = '{}/multifile_{}'.format(path, self.robotName)
            outFile = '{}/{}.proto'.format(newPath, self.robotName)
        else:
            newPath = os.path.dirname(outFile)
        mkdirSafe(newPath)  # make a dir called 'x_meshes'
        mkdirSafe(outFile.replace('.proto', '') + '_meshes')  # make a dir called 'x_meshes'
        self.meshFilesPath = outFile.replace('.proto', '') + '_meshes'
        self.f = open(inFile)
        self.pf = open(outFile, 'w')
        self.shapeIndex = 0

        indent = '  '
        level = 0
        while True:
            line = self.f.readline()
            ln = line.split()
            # termination condition:
            eof = 0
            while ln == []:
                line = self.f.readline()
                ln = line.split()
                eof += 1
                if eof > 10:
                    print('done parsing')
                    self.pf.close()
                    return
            if 'IndexedFaceSet' in ln:
                shapeLevel = 1
                newProtoString = shapeLevel * indent + ' '.join(ln[-2:]) + '\n'
                defString = ''
                if 'DEF' in ln:
                    defString = 'DEF ' + ln[ln.index('DEF') + 1]
                shapeLevel = 2
                while shapeLevel > 1:
                    line = self.f.readline()
                    ln = line.split()
                    if '}' in ln or ']' in ln:
                        shapeLevel -= 1
                    newProtoString += shapeLevel * indent + ' '.join(ln) + '\n'
                    if '{' in ln or '[' in ln:
                        shapeLevel += 1
                replaceString = self.createProto(newProtoString)
                self.pf.write(indent * level + defString + 'geometry ' + replaceString)
                self.pf.write(indent * level + '}\n')
            else:
                if '}' in ln or ']' in ln:
                    level -= 1
                elif '{' in ln or '[' in ln:
                    level += 1
                self.pf.write(line)

    def convert_all(self, sourcePath):
        outPath = sourcePath + '/multi_file_conversioin'
        mkdirSafe(outPath)
        # Find all the proto files, and create the corresponding proto filePaths
        os.chdir(sourcePath)
        # Walk the tree.
        proto_files = []  # List of the full filepaths.
        for root, directories, files in os.walk('./'):
            for filename in files:
                # Join the two strings in order to form the full filepath.
                if filename.endswith(".proto"):
                    filepath = os.path.join(root, filename)
                    filepath = filepath[1:]
                    proto_files.append(filepath)
        for proto in proto_files:
            inFile = sourcePath + proto
            outFile = outPath + proto
            self.convert(inFile, outFile)


if __name__ == "__main__":
    optParser = optparse.OptionParser(usage='usage: %prog  [options]')
    optParser.add_option('--input', dest='inPath', default=None,
                         help='Specifies the proto file, or a directory. Converts all .proto files, if it is a directory.')
    options, args = optParser.parse_args()

    inPath = options.inPath
    p2m = proto2multi()
    if os.path.basename(inPath).split('.')[-1] == 'proto':
        p2m.convert(inPath)
    else:
        inPath = os.path.abspath(inPath)
        p2m.convert_all(inPath)
