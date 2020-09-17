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
#


# Automatic PROTO to multi-file PROTO trimesh extractor
#
# INSTRUCTIONS:
# Call the script with the --input=<path> argument
#
# 1. if <path> ends in <filename.proto>:
#   - creates a new folder in the same directory named "multifile_<filename>"
#   - creates a new <filename.proto> inside this folder with all trimeshes
#     replaced by proto files, placed in a subfolder.
#   - the mesh proto files in the subfolder have the same header as the
#     original file, with the additional 'hidden' tag added.
#
# 2. If <path> does not end in ".proto" -> assumes directory
#   - creates a backup folder next to our chosen <path>
#   - searches for every .proto file in <path> recursively and:
#       - ignores .proto file with either no header or a 'hidden' tag
#       - replaces the file with a version, where meshes are extracted and put
#         into a "<filename>_meshes" folder.
#       - the mesh proto files in the subfolder have the same header as the
#         original file, with the additional 'hidden' tag added.
#
#


import os
import optparse
import shutil


class proto2multi():
    def __init__(self):
        print('Proto 2 multi-file proto converter by Simon Steinmann')

    def header(self, proto):
        """Specify VRML file header."""
        self.headerString += '# tags: hidden\n'
        self.headerString += '# This is a proto file for Webots for the %s\n\n' % (self.robotName)
        proto.write(self.headerString)

    def createProto(self, string):
        """turn mesh to proto file and stores it in the _meshes subfolder"""
        name = self.robotName + '_' + str(self.shapeIndex)
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
        self.robotName = os.path.splitext(os.path.basename(inFile))[0]
        if outFile is None:
            newPath = '{}/multifile_{}'.format(path, self.robotName)
            outFile = '{}/{}.proto'.format(newPath, self.robotName)
        else:
            newPath = os.path.dirname(outFile)
        os.makedirs(newPath, exist_ok=True)
        # make a dir called 'x_meshes'
        os.makedirs(outFile.replace('.proto', '') + '_meshes', exist_ok=True)
        self.meshFilesPath = outFile.replace('.proto', '') + '_meshes'
        self.f = open(inFile)
        self.pf = open(outFile, 'w')
        self.shapeIndex = 0
        self.headerString = ''
        indent = '  '
        level = 0
        headerExtract = True
        n = 1
        while True:
            line = self.f.readline()
            ln = line.split()
            if headerExtract:
                if line[0:1] != '#':
                    print('skipping - proto file has no header ' + outFile)
                    self.pf.close()
                    self.cleanup(inFile, outFile)
                    return
                while line.startswith('#'):
                    self.pf.write(line)
                    if line[:5] in ['#VRML', '# lic', '# doc']:
                        self.headerString += line
                    line = self.f.readline()
                    ln = line.split()
                    n += 1
                headerExtract = False
            # termination condition:
            eof = 0
            while ln == []:
                self.pf.write(line)
                line = self.f.readline()
                ln = line.split()
                eof += 1
                if eof > 10:
                    self.cleanup(inFile)
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
                self.pf.write(indent * level + defString +
                              'geometry ' + replaceString)
                self.pf.write(indent * level + '}\n')
            else:
                if '}' in ln or ']' in ln:
                    level -= 1
                elif '{' in ln or '[' in ln:
                    level += 1
                self.pf.write(line)

    def cleanup(self, inFile, outFile=None):
        if inFile[-5:] == '_temp':
            os.remove(inFile)
        if outFile is not None:
            os.remove(outFile)

    def convert_all(self, sourcePath):
        self.create_backup(sourcePath)
        outPath = sourcePath
        os.makedirs(outPath, exist_ok=True)
        # Find all the proto files, and store their filePaths
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
            print('converting ' + outFile)
            # make a copy of our inFile, which will be read and later deleted
            shutil.copy(inFile, inFile + '_temp')
            inFile = inFile + '_temp'
            self.convert(inFile, outFile)

    def create_backup(self, sourcePath):
        # Create a backup of the folder we are converting
        backupName = os.path.basename(sourcePath) + '_backup_0'
        backupPath = os.path.dirname(sourcePath) + '/' + backupName
        n = 0
        while os.path.isdir(backupPath):
            n += 1
            backupPath = backupPath[:-1] + str(n)
        shutil.copytree(sourcePath,  backupPath)


if __name__ == "__main__":
    optParser = optparse.OptionParser(usage='usage: %prog  [options]')
    optParser.add_option('--input', dest='inPath', default=None,
                         help='Specifies the proto file, or a directory. Converts all .proto files, if it is a directory.')
    options, args = optParser.parse_args()
    inPath = options.inPath
    if inPath is not None:
        p2m = proto2multi()
        if os.path.splitext(inPath)[1] == '.proto':
            p2m.convert(inPath)
            print('Multi-file extraction done')
        elif os.path.isdir(inPath):
            inPath = os.path.abspath(inPath)
            p2m.convert_all(inPath)
            print('Multi-file extraction done')
        else:
            print('ERROR: --input has to be a .proto file or directory!')
    else:
        print('Mandatory argument --input=<path> missing!\nSpecify a .proto file or directory path.')
