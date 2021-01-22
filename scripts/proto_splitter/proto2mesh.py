#!/usr/bin/env python3

# Copyright 1996-2021 Cyberbotics Ltd.
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
#   - duplicates our chosen <path> and appends "_multiProto_0" to the name
#   - the original <path> remains unchanged. If executed multiple times, the
#     trailing number for the <path> duplicates increases.
#   - searches for every .proto file in the duplicated <path> recursively and:
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
import numpy as np


class Mesh:
    def __init__(self, name, coord, coordIndex, texCoord, texCoordIndex, normal, normalIndex, creaseAngle):
        self.name = name
        self.coord = np.array(coord.replace(',', '').split(), dtype=float).reshape(-1, 3).tolist()
        faces = coordIndex.replace(',', '').split('-1')
        self.type = 'v'
        self.coordIndex = []
        for face in faces:
            self.coordIndex.append(np.array(face.split(), dtype=int).tolist())
        if len(self.coordIndex[-1]) == 0:
            self.coordIndex.pop()
        self.n_faces = len(self.coordIndex)
        self.texCoordIndex = []
        if texCoord is not None:
            self.type += 't'
            self.texCoord = np.array(texCoord.replace(',', '').split(), dtype=float).reshape(-1, 2).tolist()
            faces = texCoordIndex.replace(',', '').split('-1')
            for face in faces:
                self.texCoordIndex.append(np.array(face.split(), dtype=int).tolist())
            if len(self.texCoordIndex[-1]) == 0:
                self.texCoordIndex.pop()
            if len(self.texCoordIndex) != self.n_faces:
                print('coordIndex and texCoordIndex mismatch: ' + str(len(self.texCoordIndex)) + ' != ' + str(self.n_faces))
        else:
            self.texCoord = []
        self.normalIndex = []
        if normal is not None:
            self.type += 'n'
            self.normal = np.array(normal.replace(',', '').split(), dtype=float).reshape(-1, 3).tolist()
            faces = normalIndex.replace(',', '').split('-1')
            for face in faces:
                self.normalIndex.append(np.array(face.split, dtype=int).tolist())
            if len(self.normalIndex[-1]) == 0:
                self.normalIndex.pop()
            if len(self.normalIndex) != self.n_faces:
                print('coordIndex and normalIndex mismatch')
        else:
            self.normal = []
        self.creaseAngle = float(creaseAngle)

    def remove_duplicate_texture_coordinates(self):
        newTexCoord = []
        for i, texCoord in enumerate(self.texCoord):
            try:
                j = self.texCoord.index(texCoord, i + 1)
                for index in self.texCoordIndex:
                    for k, v in enumerate(index):
                        if v == j:
                            index[k] = i
                        elif v > j:
                            index[k] = v - 1
            except ValueError:
                newTexCoord.append(texCoord)
                continue
        self.texCoord = newTexCoord

    def apply_crease_angle(self):
        if self.creaseAngle == 0:
            return
        if len(self.normal) > 0:
            return
        if self.type[-1] == 'n':
            return
        faceNormal = []
        for counter, face in enumerate(self.coordIndex):
            size = len(face)
            if size < 3:
                print('Bad face with ' + str(size) + ' vertices.')
                continue
            p0 = [self.coord[face[0]][0], self.coord[face[0]][1], self.coord[face[0]][2]]
            p1 = [self.coord[face[1]][0], self.coord[face[1]][1], self.coord[face[1]][2]]
            p2 = [self.coord[face[2]][0], self.coord[face[2]][1], self.coord[face[2]][2]]
            n = np.cross(np.subtract(p1, p0), np.subtract(p2, p0))
            normalized = n / np.sqrt(np.sum(n**2))
            faceNormal.append(normalized)

        faceIndex = [[] for _ in range(len(self.coord))]
        for counter, face in enumerate(self.coordIndex):
            for index in face:
                faceIndex[index].append(counter)

        counter = 0
        for i, face in enumerate(self.coordIndex):
            self.normalIndex.append([])
            for j in face:
                n = faceNormal[i]
                creased = False
                for k in faceIndex[j]:
                    if k == i:
                        continue
                    angle = np.arccos(np.clip(np.dot(faceNormal[i], faceNormal[k]), -1.0, 1.0))
                    if angle < self.creaseAngle:
                        n = np.add(n, faceNormal[k])
                        creased = True
                if creased:
                    n = n / np.sqrt(np.sum(n**2))
                try:
                    index = self.normal.index(n.tolist())
                    self.normalIndex[i].append(index)
                except ValueError:
                    self.normal.append(n.tolist())
                    self.normalIndex[i].append(counter)
                    counter += 1

        self.type += 'n'
        self.creaseAngle = 0

    def write_obj(self, file):
        file.write('o ' + self.name + '\n')
        for vertex in self.coord:
            file.write('v {} {} {}\n'.format(vertex[0], vertex[1], vertex[2]))
        # texture coordinates
        for vt in self.texCoord:
            file.write('vt {} {}\n'.format(vt[0], 1 - vt[1]))
        # normal coordinates
        for vn in self.normal:
            file.write('vn {} {} {}\n'.format(vn[0], vn[1], vn[2]))
        for n in range(self.n_faces):
            size = len(self.coordIndex[n])
            file.write('f')
            for i in range(size):
                if self.type == 'v':
                    file.write(' {}'.format(self.coordIndex[n][i] + 1))
                if self.type == 'vt':
                    file.write(' {}/{}'.format(self.coordIndex[n][i] + 1, self.texCoordIndex[n][i] + 1))
                if self.type == 'vn':
                    file.write(' {}//{}'.format(self.coordIndex[n][i] + 1, self.normalIndex[n][i] + 1))
                if self.type == 'vtn':
                    file.write(' {}/{}/{}'.format(self.coordIndex[n][i] + 1, self.texCoordIndex[n][i] + 1,
                                                  self.normalIndex[n][i] + 1))
            file.write('\n')


class proto2mesh:
    def __init__(self):
        print('Proto 2 multi-file proto converter by Simon Steinmann')

    def get_data_from_field(self, ln):
        i = ln.index('[')
        data = ' '.join(ln[i:])
        while ']' not in ln:
            line = self.f.readline()
            ln = line.split()
            data += line
        data = ' '.join(data.split())
        data = data.replace('[', '').replace(']', '')
        return ln, data

    def convert(self, inFile, outFile=None):
        path = os.path.dirname(inFile)
        self.robotName = os.path.splitext(os.path.basename(inFile))[0]
        if outFile is None:
            newPath = '{}/{}_multifile'.format(path, self.robotName)
            outFile = '{}/{}.proto'.format(newPath, self.robotName)
        else:
            newPath = os.path.dirname(outFile)
        os.makedirs(newPath, exist_ok=True)
        # make a dir called 'x_meshes'
        os.makedirs(outFile.replace('.proto', '') + '_meshes', exist_ok=True)
        self.meshFilesPath = outFile.replace('.proto', '') + '_meshes'
        # The input PROTO file we are converting
        self.f = open(inFile)
        self.protoFileString = ''
        # The new proto file, with meshes extracted
        self.pf = open(outFile, 'w')
        self.shapeIndex = 0
        # Stores the DEF of the closest related parentnode of Type 'Group', 'Transform' or 'Shape'.
        # If a mesh has no name, this is used instead.
        parentDefName = None
        # A dictionary, which will get filled with all meshes of the PROTO file. Each
        #  mesh has a key '<level>_<meshID>' level is the indent, meshID is unique
        #  number, counting up from 0. The value is an instance of the Mesh class.
        meshes = {}
        meshID = 0
        indent = '  '
        level = 0
        while True:
            line = self.f.readline()
            ln = line.split()
            # termination condition:
            eof = 0
            while ln == []:
                self.protoFileString += line
                line = self.f.readline()
                ln = line.split()
                eof += 1
                if eof > 10:
                    self.f.close()
                    self.cleanup(inFile)
                    for mesh in meshes.values():
                        # mesh.remove_duplicate_texture_coordinates()
                        mesh.apply_crease_angle()
                    self.write_obj(meshes)
                    self.pf.write(self.protoFileString)
                    self.pf.close()
                    return
            if 'name' in ln:
                name = ln[ln.index('name') + 1].replace('"', '')
                if name == 'IS':
                    name = 'base_link'
                counter = 0
                for k, v in meshes.items():
                    if v.name is None:
                        mlvl = int(k.split('_')[0])
                        if mlvl in [level + 2, level + 4]:
                            v.name = name + '_' + str(counter)
                            counter += 1
            if 'DEF' in ln:
                if 'Group' in ln or 'Transform' in ln or 'Shape' in ln:
                    parentDefName = str(level) + '_' + ln[ln.index('DEF') + 1]
            if 'IndexedFaceSet' in ln:
                coord = coordIndex = texCoord = texCoordIndex = normal = normalIndex = creaseAngle = name = None
                defString = ''
                if 'DEF' in ln:
                    defString = 'DEF ' + ln[ln.index('DEF') + 1] + ' '
                    name = ln[ln.index('DEF') + 1]
                elif parentDefName is not None:
                    name = parentDefName.split('_')[1]
                shapeLevel = 1
                meshID += 1
                while shapeLevel > 0:
                    if 'coord' in ln:
                        line = self.f.readline()
                        ln = line.split()
                        ln, coord = self.get_data_from_field(ln)
                    if 'texCoord' in ln:
                        line = self.f.readline()
                        ln = line.split()
                        ln, texCoord = self.get_data_from_field(ln)
                    if 'normal' in ln:
                        line = self.f.readline()
                        ln = line.split()
                        ln, normal = self.get_data_from_field(ln)
                    if 'coordIndex' in ln:
                        ln, coordIndex = self.get_data_from_field(ln)
                    if 'texCoordIndex' in ln:
                        ln, texCoordIndex = self.get_data_from_field(ln)
                    if 'normalIndex' in ln:
                        ln, normalIndex = self.get_data_from_field(ln)
                    if 'creaseAngle' in ln:
                        creaseAngle = ln[ln.index('creaseAngle') + 1]
                    line = self.f.readline()
                    ln = line.split()
                    if '}' in ln:
                        shapeLevel -= 1
                    if '{' in ln:
                        shapeLevel += 1
                key = str(level) + '_' + str(meshID)
                meshes[key] = Mesh(name, coord, coordIndex, texCoord, texCoordIndex, normal, normalIndex, creaseAngle)
                parentDefName = None
                self.protoFileString += indent * (level + 1) + 'geometry ' + defString + 'Mesh {\n'
                self.protoFileString += indent * (level + 2) + 'url MeshID_' + key + '_placeholder\n'
                self.protoFileString += indent * (level + 1) + '}\n'
            else:
                if '}' in ln or ']' in ln:
                    level -= 1
                    if parentDefName is not None:
                        if level < int(parentDefName.split('_')[0]):
                            parentDefName = None
                elif '{' in ln or '[' in ln:
                    level += 1
                # Write the whole line from input to output file, without changes
                self.protoFileString += line

    def cleanup(self, inFile, outFile=None):
        if inFile.endswith('_temp'):
            os.remove(inFile)
        if outFile is not None:
            os.remove(outFile)

    def convert_all(self, sourcePath):
        outPath = self.create_outputDir(sourcePath)
        os.makedirs(outPath, exist_ok=True)
        # Find all the proto files, and store their filePaths
        os.chdir(sourcePath)
        # Walk the tree.
        protoFiles = []  # List of the full filepaths.
        for root, directories, files in os.walk('./'):
            for filename in files:
                # Join the two strings in order to form the full filepath.
                if filename.endswith('.proto'):
                    filepath = os.path.join(root, filename)
                    filepath = filepath[1:]
                    protoFiles.append(filepath)
        for proto in protoFiles:
            inFile = sourcePath + proto
            outFile = outPath + proto
            print('converting ' + outFile)
            # make a copy of our inFile, which will be read and later deleted
            shutil.copy(inFile, inFile + '_temp')
            inFile = inFile + '_temp'
            self.convert(inFile, outFile)

    def create_outputDir(self, sourcePath):
        # Create a new directory, where the convrted files will be stored.
        newDirName = os.path.basename(sourcePath) + '_multiProto_0'
        newDirPath = os.path.dirname(sourcePath) + '/' + newDirName
        n = 0
        while os.path.isdir(newDirPath):
            n += 1
            newDirPath = newDirPath[:-1] + str(n)
        shutil.copytree(sourcePath, newDirPath)
        return newDirPath

    def compute_normals(self, meshes):
        return

    def write_obj(self, meshes):
        for k, mesh in meshes.items():
            # Replace the placholder ID of the generated .obj meshes with their path
            searchString = 'MeshID_' + k + '_placeholder'
            replaceString = '"' + self.robotName + '_meshes/' + mesh.name + '.obj"'
            self.protoFileString = self.protoFileString.replace(searchString, replaceString)
            # Create a new .obj mesh file
            filepath = '{}/{}.obj'.format(self.meshFilesPath, mesh.name)
            f = open(filepath, 'w')
            mesh.write_obj(f)
            f.close()


if __name__ == '__main__':
    optParser = optparse.OptionParser(usage='usage: %prog  [options]')
    optParser.add_option(
        '--input',
        dest='inPath',
        default=None,
        help='Specifies the proto file, or a directory. Converts all .proto files, if it is a directory.',
    )
    options, args = optParser.parse_args()
    inPath = options.inPath
    if inPath is not None:
        p2m = proto2mesh()
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
        print(
            'Mandatory argument --input=<path> missing!\nSpecify a .proto file or directory path.'
        )
