#!/usr/bin/env python3

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
#
# Author Simon Steinmann (https://github.com/Simon-Steinmann)
#


# Automatic PROTO to multi-file PROTO trimesh extractor
#
# INSTRUCTIONS:
# Call the script with the --input=<path> argument
#
# 1. if <path> ends in <filename>.proto:
#   - creates a new folder in the same directory named "multifile_<filename>"
#   - creates a new <filename>.proto inside this folder with all trimeshes
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
#         into a "<protoname>/" folder.
#       - the mesh proto files in the subfolder have the same header as the
#         original file, with the additional 'hidden' tag added.
#
#


import copy
import os
import argparse
import shutil
import numpy as np
import sys
import time

# modules needed for multithreading
import multiprocessing
import signal


def mutliprocess_initializer():
    """Ignore CTRL+C in the worker process."""
    signal.signal(signal.SIGINT, signal.SIG_IGN)


class Mesh:
    def __init__(self, name, coord, coordIndex, texCoord, texCoordIndex, normal, normalIndex, creaseAngle, verbose=True):
        self.verbose = verbose
        self.name = name
        self.coord = np.array(coord.replace(',', '').split(), dtype=float).reshape(-1, 3).tolist()
        faces = coordIndex.replace(',', '').split('-1')
        self.type = 'v'
        self.coordIndex = self.faces_to_coordIndex(faces)
        if len(self.coordIndex[-1]) == 0:
            self.coordIndex.pop()
        self.n_faces = len(self.coordIndex)
        self.normalIndex = self.texCoordIndex = []
        if texCoord is not None:
            self.type += 't'
            self.texCoord = np.array(texCoord.replace(',', '').split(), dtype=float).reshape(-1, 2).tolist()
            if texCoordIndex is None:
                self.texCoordIndex = copy.deepcopy(self.coordIndex)
            else:
                faces = texCoordIndex.replace(',', '').split('-1')
                self.texCoordIndex = self.faces_to_coordIndex(faces)
            if len(self.texCoordIndex[-1]) == 0:
                self.texCoordIndex.pop()
            if len(self.texCoordIndex) != self.n_faces:
                raise Exception('texCoordIndex and coordIndex mismatch: ' +
                                str(len(self.texCoordIndex)) + ' != ' + str(self.n_faces))
        else:
            self.texCoord = []
        if normal is not None:
            self.type += 'n'
            self.normal = np.array(normal.replace(',', '').split(), dtype=float).reshape(-1, 3).tolist()
            if normalIndex is None:
                self.normalIndex = copy.deepcopy(self.coordIndex)
            else:
                faces = normalIndex.replace(',', '').split('-1')
                self.normalIndex = self.faces_to_coordIndex(faces)
            if len(self.normalIndex[-1]) == 0:
                self.normalIndex.pop()
            if len(self.normalIndex) != self.n_faces:
                raise Exception('normalIndex and coordIndex mismatch: ' +
                                str(len(self.normalIndex)) + ' != ' + str(self.n_faces))
        else:
            self.normal = []
        self.creaseAngle = float(creaseAngle)

    def faces_to_coordIndex(self, faces):
        """Converts a list of faces into a list of triangle faces and returns type=int coordIndex"""
        coordIndex = []
        for face in faces:
            f = face.split()
            if len(f) == 3:
                coordIndex.append(np.array(f, dtype=int).tolist())
            elif len(f) == 4:
                coordIndex.append(np.array([f[0], f[1], f[2]], dtype=int).tolist())
                coordIndex.append(np.array([f[0], f[2], f[3]], dtype=int).tolist())
            elif len(f) != 0:
                raise Exception("ERROR: only faces with 3 or 4 vertices can be converted. Face causing error: ", f)
        return coordIndex

    def remove_duplicate(self, type):
        if type == 'vertex':
            coord = self.coord
            coordIndex = self.coordIndex
        elif type == 'texture':
            coord = self.texCoord
            coordIndex = self.texCoordIndex
        elif type == 'normal':
            coord = self.normal
            coordIndex = self.normalIndex
        if len(coord) == 0:
            if len(coordIndex) != 0:
                raise Exception(f'Error: wrong {type} index')
            return

        # first pass: remove indices to duplicate values
        if self.verbose:
            print('removing duplicate ' + type)
        # returns array of unique coords, "indices" array with duplicate indices replaced by unique ones
        # and a count array, containing number of duplicates per index. Look up np.unique() documentation for more.
        uniqueCoord, indices, counts = np.unique(coord, return_inverse=True, return_counts=True, axis=0)
        removed = np.sum(counts)
        for index in coordIndex:
            for k, v in enumerate(index):
                index[k] = indices[v]
        if removed > 0 and self.verbose:
            print(f'    Removed {removed} duplicate {type} coordinates', flush=True)

        # second pass: remove unused vertices and adjust indices
        uniqueIndices = np.unique(coordIndex)  # array of all used indices
        indexShift = []  # index = old coordIndex, value = new coordindex (unused coords removed)
        newCoord = []
        newCoordIndex = copy.deepcopy(coordIndex)
        removed = 0
        for i, c in enumerate(uniqueCoord):
            if i in uniqueIndices:
                newCoord.append(c)
            else:
                removed += 1
            indexShift.append(i - removed)
        for index in newCoordIndex:
            for k, v in enumerate(index):
                index[k] = indexShift[v]

        if removed > 0 and self.verbose:
            print(f'    Removed {removed} unused {type} coordinates', flush=True)
        if type == 'vertex':
            self.coord = newCoord
            self.coordIndex = newCoordIndex
        elif type == 'texture':
            self.texCoord = newCoord
            self.texCoordIndex = newCoordIndex
        elif type == 'normal':
            self.normal = newCoord
            self.normalIndex = newCoordIndex

    def apply_crease_angle(self):
        if self.creaseAngle == 0:
            return
        if len(self.normal) > 0:
            return
        if self.type[-1] == 'n':
            return
        faceNormal = []
        if self.verbose:
            print('    Computing normals from creaseAngle', flush=True)
        for face in self.coordIndex[:]:  # [:] creates a copy
            size = len(face)
            if size < 3:
                raise Exception('Bad face with ' + str(size) + ' vertices.')
            p0 = [self.coord[face[0]][0], self.coord[face[0]][1], self.coord[face[0]][2]]
            p1 = [self.coord[face[1]][0], self.coord[face[1]][1], self.coord[face[1]][2]]
            p2 = [self.coord[face[2]][0], self.coord[face[2]][1], self.coord[face[2]][2]]
            n = np.cross(np.subtract(p1, p0), np.subtract(p2, p0))
            k = np.sqrt(np.sum(n**2))
            if k == 0:
                if self.verbose:
                    print('Wrong face: ' + str(face[0]) + ', ' + str(face[1]) + ', ' + str(face[2]) + ', -1\n' +
                          str(p0) + str(p1) + str(p2) + '\n' + str(n), flush=True)
                self.coordIndex.remove(face)
                continue
            normalized = n / k
            faceNormal.append(normalized)
        if self.n_faces > len(self.coordIndex):
            if self.verbose:
                print('Invalid faces removed: ', self.n_faces - len(self.coordIndex))
            self.n_faces = len(self.coordIndex)
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
                n = np.around(n, 4)
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
            file.write('vt {} {}\n'.format(vt[0], vt[1]))
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
        print('Proto 2 mesh proto converter by Simon Steinmann & Olivier Michel', flush=True)
        self.disableMeshOptimization = False
        self.disableFileCreation = False

    def get_data_from_field(self, ln):
        defName = None
        if 'DEF' in ln:
            defName = ln[ln.index('DEF') + 1]
        if 'USE' in ln:
            useName = ln[ln.index('USE') + 1]
            line = self.f.readline().split('#')[0]
            if '{' in line:
                self.shapeLevel += 1
            self.lineNumber += 1
            ln = line.split()
            # print(useName, len(self.meshDEFcache[useName]))
            return ln, self.meshDEFcache[useName]
        line = ' '.join(ln).split('#')[0]
        while '[' not in line:
            line = self.f.readline().split('#')[0]
            self.lineNumber += 1
            ln = line.split()
            if '{' in line:
                self.shapeLevel += 1
        i = ln.index('[')
        data = ' '.join(ln[i:])
        while ']' not in line:
            line = self.f.readline().split('#')[0]
            if '{' in line:
                self.shapeLevel += 1
            self.lineNumber += 1
            if '%' in line:
                raise Exception("ERROR: LUA script for mesh data is not supported.")
            ln = line.split()
            data += line
        data = ' '.join(data.split())
        data = data.replace('[', '').replace(']', '')
        if defName is not None:  # store coord and coordIndex data for DEF to assign to USE later
            self.meshDEFcache[defName] = data
        return ln, data

    def convert(self, inFile, outFile=None, verbose=True):
        path = os.path.dirname(inFile)
        self.robotName = os.path.splitext(os.path.basename(inFile))[0]
        if outFile is None:
            newPath = '{}/{}_multifile'.format(path, self.robotName)
            outFile = '{}/{}.proto'.format(newPath, self.robotName)
        else:
            newPath = os.path.dirname(outFile)
        print('Converting ' + outFile, flush=True)
        os.makedirs(newPath, exist_ok=True)
        # make a directory to store meshes with the same name as the proto
        os.makedirs(outFile.replace('.proto', ''), exist_ok=True)
        self.meshFilesPath = outFile.replace('.proto', '')
        # The input PROTO file we are converting
        self.f = open(inFile)
        self.protoFileString = ''
        # The new proto file, with meshes extracted
        if not self.disableFileCreation:
            self.pf = open(outFile, 'w', newline='\n')
        self.shapeIndex = 0
        # Stores the DEF of the closest related parentnode of Type 'Group', 'Transform' or 'Shape'.
        # If a mesh has no name, this is used instead.
        parentDefName = None
        # A dictionary, which will get filled with all meshes of the PROTO file. Each
        #  mesh has a key '<level>_<meshID>' level is the indent, meshID is unique
        #  number, counting up from 0. The value is an instance of the Mesh class.
        self.lineNumber = 0  # current line in the source proto file. For debug purposes.
        meshes = {}
        self.meshDEFcache = {}  # stores coord and coordIndex data that have DEF to assign to USE later
        meshID = 0
        indent = '  '
        level = 0
        while True:
            line = self.f.readline()
            self.lineNumber += 1
            ln = line.split()
            # termination condition:
            eof = 0
            while ln == []:
                self.protoFileString += line
                line = self.f.readline()
                self.lineNumber += 1
                ln = line.split()
                eof += 1
                if eof > 10:
                    self.f.close()
                    self.cleanup(inFile)
                    total = len(meshes)
                    count = 1
                    counter = 0
                    # If the mesh has no name, we use a generic 'Mesh' name.
                    # This can occur, when the parent Solid has no name.
                    for k, v in meshes.items():
                        if v.name is None or '%<=' in v.name:
                            v.name = 'Mesh' + str(counter)
                            counter += 1
                    for mesh in meshes.values():
                        nc = str(len(mesh.coordIndex))
                        mesh.verbose = verbose
                        if verbose:
                            print('  Processing mesh ' + mesh.name + ' (' + str(count) +
                                  '/' + str(total) + ') n-verticies: ', nc, flush=True)
                        count += 1
                        if not self.disableMeshOptimization:
                            mesh.remove_duplicate('vertex')
                            mesh.remove_duplicate('texture')
                            mesh.apply_crease_angle()
                            mesh.remove_duplicate('normal')
                    if not self.disableFileCreation:
                        self.write_obj(meshes)
                        self.pf.write(self.protoFileString)
                        self.pf.close()
                    return 'success'
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
                    name = parentDefName[parentDefName.index('_') + 1:]
                self.shapeLevel = 1
                meshID += 1
                while self.shapeLevel > 0:
                    if 'coord' in ln:
                        ln, coord = self.get_data_from_field(ln)
                    if 'texCoord' in ln:
                        ln, texCoord = self.get_data_from_field(ln)
                    if 'normal' in ln:
                        ln, normal = self.get_data_from_field(ln)
                    if 'coordIndex' in ln:
                        ln, coordIndex = self.get_data_from_field(ln)
                    if 'texCoordIndex' in ln:
                        ln, texCoordIndex = self.get_data_from_field(ln)
                    if 'normalIndex' in ln:
                        ln, normalIndex = self.get_data_from_field(ln)
                    if 'creaseAngle' in ln:
                        creaseAngle = ln[ln.index('creaseAngle') + 1]
                    else:
                        creaseAngle = 0
                    line = self.f.readline()
                    self.lineNumber += 1
                    ln = line.split()
                    if '}' in ln:
                        self.shapeLevel -= 1
                    if '{' in ln:
                        self.shapeLevel += 1
                key = str(level) + '_' + str(meshID)
                if name is not None:
                    name = name.lower()
                meshes[key] = Mesh(name, coord, coordIndex, texCoord, texCoordIndex, normal, normalIndex, creaseAngle)
                parentDefName = None
                self.protoFileString += indent * level + 'geometry ' + defString + 'Mesh {\n'
                self.protoFileString += indent * (level + 1) + 'url MeshID_' + key + '_placeholder\n'
                self.protoFileString += indent * level + '}\n'
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
        if inFile.endswith('proto_temp'):
            os.remove(inFile)
        if outFile is not None:
            os.remove(outFile)

    def convert_all(self, pool, sourcePath, outPath, verbose):

        if outPath is None:
            outPath = sourcePath
        outPath = self.create_outputDir(sourcePath, outPath)
        print(outPath)
        if not self.disableFileCreation:
            os.makedirs(outPath, exist_ok=True)
        # Find all the proto files, and store their filePaths
        os.chdir(sourcePath)
        # Walk the tree.
        self.protoFiles = []  # List of the full filepaths.
        failedConvertions = []
        for root, directories, files in os.walk('./'):
            for filename in files:
                # Join the two strings in order to form the full filepath.
                if filename.endswith('.proto'):
                    filepath = os.path.join(root, filename)
                    filepath = filepath[1:]
                    self.protoFiles.append(filepath)
        for proto in self.protoFiles:
            inFile = outPath + proto
            outFile = outPath + proto
            # make a copy of our inFile, which will be read and later deleted
            shutil.copy(inFile, inFile + '_temp')
            inFile = inFile + '_temp'
            if pool is None:
                try:
                    self.convert(inFile, outFile, verbose)
                except Exception as e:
                    failedConvertions.append(sourcePath + proto + ' - line: ' + str(self.lineNumber) + '\n' + str(e))
                    self.move_failed_conversions(sourcePath, outPath, proto, e)
            else:
                multiprocessing.active_children()
                r = pool.apply_async(self.convert, args=(inFile, outFile, verbose))
                failedConvertions.append([r, sourcePath + proto + '\n', sourcePath, outPath, proto])
        return failedConvertions

    def move_failed_conversions(self, sourcePath, outPath, proto, e):
        # create a _FAILED_CONVERSIONS folder inside our output path
        failedBasePath = os.path.join(outPath, '_FAILED_CONVERSIONS')
        outFile = os.path.abspath(outPath + proto)
        # deleta all files, which have been created for the failed conversion
        try:
            self.f.close()
            os.remove(outFile)
        except OSError:
            pass
        try:
            os.remove(outFile + '_temp')
        except OSError:
            pass
        meshFolder = outFile.replace('.proto', '')
        if os.path.isdir(meshFolder):
            shutil.rmtree(meshFolder)
        # copy the original source .proto file with its directory tree
        # into the _FAILED_CONVERSIONS folder.
        os.makedirs(failedBasePath + proto, exist_ok=True)
        with open(failedBasePath + proto + "_ErrorLog.txt", "w") as text_file:
            text_file.write(str(e))
        shutil.copy(sourcePath + proto, failedBasePath + proto)

    def create_outputDir(self, sourcePath, outPath):
        # Create a new directory, where the convrted files will be stored.
        os.makedirs(outPath, exist_ok=True)
        if sourcePath == outPath:
            newDirName = os.path.basename(sourcePath) + '_multiProto_0'
            newDirPath = os.path.join(os.path.dirname(sourcePath), newDirName)
        else:
            newDirPath = os.path.join(outPath, os.path.basename(sourcePath))
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
            replaceString = '"' + self.robotName + '/' + mesh.name + '.obj"'
            self.protoFileString = self.protoFileString.replace(searchString, replaceString)
            # Create a new .obj mesh file
            filepath = '{}/{}.obj'.format(self.meshFilesPath, mesh.name)
            f = open(filepath, 'w', newline='\n')
            mesh.write_obj(f)
            f.close()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument(
        dest='input',
        default=None,
        help='Specifies the proto file, or a directory. Converts all .proto files, if it is a directory.',
    )
    parser.add_argument(
        '--output',
        dest='outPath',
        default=None,
        help='Specifies the output path. Only for directory conversions.',
    )
    parser.add_argument(
        '-m', '--multithreaded',
        dest='multithreaded',
        default=False,
        action='store_true',
        help='If set, enables multicore processing for directories with several PROTO files. May not work on Windows.',
    )
    parser.add_argument(
        '-v', '--verbose',
        dest='verbose',
        default=False,
        action='store_true',
        help='If set, detailed output of mesh conversion is shown in console.',
    )
    parser.add_argument(
        '--check-protos-validity', '--cpv',
        dest='checkProtoValidity',
        default=False,
        action='store_true',
        help='If set, will quickly go through all protos files and output any errors. No mesh calculations or files created.',
    )
    args = parser.parse_args()
    inPath = args.input
    outPath = args.outPath
    verbose = args.verbose
    checkProtoValidity = args.checkProtoValidity
    multithreaded = args.multithreaded
    tStart = time.time()
    p2m = proto2mesh()
    if checkProtoValidity:
        p2m.disableMeshOptimization = True
        p2m.disableFileCreation = True
        multithreaded = False  # disable multithreading, otherwise error collection does not work.
    if multithreaded:
        pool = multiprocessing.Pool(initializer=mutliprocess_initializer, maxtasksperchild=10)
    else:
        pool = None

    try:
        if inPath is not None:
            if os.path.splitext(inPath)[1] == '.proto':
                p2m.convert(inPath, verbose=verbose)
                print('Conversion done. Duration: ', round(time.time() - tStart, 3), ' seconds')
            elif os.path.isdir(inPath):
                inPath = os.path.abspath(inPath)
                results = p2m.convert_all(pool, inPath, outPath, verbose)
                if multithreaded:
                    pool.close()
                    pool.join()
                    print('\nFailed conversions. Check PROTO formatting:\n')
                    for i in range(len(results)):
                        r = results[i]
                        try:
                            r[0].get()
                        except Exception as e:
                            p2m.move_failed_conversions(r[2], r[3], r[4], e)
                            print('\n', r[1], e)
                else:
                    print('\nFailed conversions. Check PROTO formatting:\n')
                    for path in results:
                        print('\n' + path)
                print('Conversion done. Duration: ', round(time.time() - tStart, 3), ' seconds')
            else:
                sys.exit('Error: --input has to be a .proto file or directory!')
        else:
            sys.exit('Error: Mandatory argument <input> is missing! It should specify a .proto file or directory path.')
    except KeyboardInterrupt:
        print('----------KeyboardInterrupt-------------------------------------------------------')
        pool.terminate()
        pool.join()
        sys.exit("KeyboardInterrupt! Terminating running processes.")
