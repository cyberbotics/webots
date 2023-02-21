#!/usr/bin/env python

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

import glob
import os
import errno
import zipfile
import shutil
import platform
import sys

xc16_version = '1.24'


def mkdir_p(path):
    try:
        os.makedirs(path)
    except OSError as exc:
        if exc.errno == errno.EEXIST and os.path.isdir(path):
            pass
        else:
            raise RuntimeError("Cannot create directory %s" % path)


def zipdir(path, zip):
    for root, dirs, files in os.walk(path):
        for file in files:
            zip.write(os.path.join(root, file))


scriptdir = os.path.dirname(os.path.realpath(__file__))
dstdir = os.path.join(scriptdir, 'xc16')

if platform.system() == 'Darwin':
    srcdir = os.path.join(os.sep, 'Applications', 'microchip', 'xc16', 'v' + xc16_version)
elif platform.system() == 'Linux':
    srcdir = os.path.join(os.sep, 'opt', 'microchip', 'xc16', 'v' + xc16_version)
elif platform.system() == 'Windows':
    srcdir = os.path.join(os.environ["PROGRAMFILES"], 'Microchip', 'xc16', 'v' + xc16_version)
else:
    raise RuntimeError('Unsupported platform')

files = [
    'bin/bin/elf-ar.exe',
    'bin/bin/elf-as.exe',
    'bin/bin/elf-bin2hex.exe',
    'bin/bin/elf-cc1.exe',
    'bin/bin/elf-gcc.exe',
    'bin/bin/elf-ld.exe',
    'bin/device_files/30F6014A.info',
    'bin/c30_device.info',
    'bin/xc16-ar.exe',
    'bin/xc16-as.exe',
    'bin/xc16-bin2hex.exe',
    'bin/xc16-cc1.exe',
    'bin/xc16-gcc.exe',
    'bin/xc16-ld.exe',
    'include/*.h',
    'lib/dsPIC30F/libp30F6014A-elf.a',
    'lib/libc-elf.a',
    'lib/libdsp-elf.a',
    'lib/libm-elf.a',
    'lib/libpic30-elf.a',
    'support/dsPIC30F/gld/p30F6014A.gld',
    'support/dsPIC30F/h/p30F6014A.h',
    'support/dsPIC30F/h/p30Fxxxx.h',
    'support/dsPIC30F/inc/p30F6014A.inc',
    'support/dsPIC30F/inc/p30Fxxxx.inc'
]

print('copying files...')
sys.stdout.flush()
for f in files:
    if platform.system() != 'Windows':
        f = f.replace('.exe', '')
    f = os.path.normpath(f)
    dstfile = os.path.join(dstdir, f)
    dstdirpath = os.path.dirname(dstfile)
    mkdir_p(dstdirpath)

    absolutepaths = glob.glob(os.path.join(srcdir, f))
    if not absolutepaths:
        raise RuntimeError('Could not find ' + os.path.join(srcdir, f))
    for path in absolutepaths:
        basename = os.path.basename(path)
        try:
            shutil.copy(path, os.path.join(dstdirpath, basename))
        except IOError as e:
            raise RuntimeError("Unable to copy file. %s" % e)


print('zipping xc16 directory...')
sys.stdout.flush()
zipf = zipfile.ZipFile('xc16-' + xc16_version + '.zip', 'w')
zipdir('xc16', zipf)
zipf.close()
print('done.')
