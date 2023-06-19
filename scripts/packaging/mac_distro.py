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

"""Generate macOS Webots package."""

from generic_distro import WebotsPackage, remove_force
import json
import os
import re
import shutil
import subprocess
import sys


def command(cmd):
    process = subprocess.Popen(cmd, stdout=subprocess.PIPE, shell=True)
    return process.communicate()[0].decode().strip()


def check_rpath(home_path):
    """Update rpaths."""
    os.chdir(home_path)
    dylibFiles = command('find . -type f -name *dylib -o -name *jnilib | grep -v dependencies | grep -v ros | grep -v sumo | '
                         'grep -v nao_soccer | grep -v dashel | grep -v build').split()
    frameworkFiles = command('find Contents -name *.framework | '
                             'sed -e "s:\\(.*\\)/\\([^/]*\\).framework:\\1/\\2.framework/\\2:"').split()
    for i in range(len(frameworkFiles)):
        frameworkFiles[i] = os.path.realpath(frameworkFiles[i]).replace(home_path, '')
        frameworkFiles[i] = re.sub(r"^\.", "", frameworkFiles[i])
        frameworkFiles[i] = re.sub(r"^/", "", frameworkFiles[i])
    controllerFiles = command('find projects resources -name controllers | '
                              'xargs -I{} find {} -maxdepth 1 -mindepth 1 -type d | '
                              'grep -v ros | '
                              'sed -e "s:\\(.*\\)/\\([^/]*\\):\\1/\\2/\\2:" | '
                              'perl -ne \'chomp(); if (-e $_) {print "$_\n"}\' ').split()
    binaryFiles = [
        'Contents/MacOS/webots'
    ]
    qtBinaryFiles = [
        'bin/qt/moc',
        'bin/qt/lrelease',
        'bin/qt/lupdate'
    ]

    success = True

    # Check dependencies are:
    # - absolute (system) and are not containing local (macports)
    # - relative to @rpath (= WEBOTS_HOME) and are existing
    for f in dylibFiles + frameworkFiles + controllerFiles + binaryFiles + qtBinaryFiles:
        dependencies = command('otool -L ' + f + ' | grep -v ' + f + ': | sed -e "s: (compatibility.*::" | '
                               'sed -e "s:^[ \t]*::"').split('\n')
        for d in dependencies:
            if d.startswith(f + ' ('):
                continue
            if (not d.startswith('/') and not d.startswith('@rpath/')) or 'local' in d:
                success = False
                sys.stderr.write('Dependency error:\n')
                sys.stderr.write('- File: ' + f + '\n')
                sys.stderr.write('- Dependency: ' + d + '\n')
            elif d.startswith('@rpath/'):
                expectedFile = d.replace('@rpath', home_path)
                if not os.path.exists(expectedFile):
                    success = False
                    sys.stderr.write('Dependency error:\n')
                    sys.stderr.write('- File: ' + f + '\n')
                    sys.stderr.write('- Dependency: ' + d + '\n')

    # check that the binaries have an RPATH
    for f in controllerFiles + binaryFiles + qtBinaryFiles:
        rpath = command('otool -l ' + f + ' | grep LC_RPATH -A 3 | grep path | cut -c15- | cut -d\' \' -f1')
        if rpath is None or not rpath:
            success = False
            sys.stderr.write('RPATH not defined in:\n')
            sys.stderr.write('- File: ' + f + '\n')
        elif not rpath.startswith('@loader_path/') and not rpath.startswith('@rpath/') and f not in qtBinaryFiles:
            success = False
            sys.stderr.write('RPATH error in:\n')
            sys.stderr.write('- File: ' + f + '\n')
            sys.stderr.write('- rpath: ' + rpath + '\n')

    if success:
        print('  Dylibs dependencies: ok')
        print('  Frameworks dependencies: ok')
    else:
        sys.exit(1)  # Quit the script with an error code.


class MacWebotsPackage(WebotsPackage):
    def __init__(self, package_name):
        super().__init__(package_name)
        self.bundle_name = package_name + '.app'
        self.package_webots_path = os.path.join(self.distribution_path, self.bundle_name)

        # remove previous distribution package
        path = os.path.join(self.distribution_path, self.bundle_name)
        remove_force(path)
        remove_force(os.path.join(self.distribution_path,
                                  f"{self.application_name_lowercase_and_dashes}-{self.package_version}.dmg"))
        os.makedirs(path)

    def create_webots_bundle(self, include_commit_file):
        super().create_webots_bundle(include_commit_file)

        print('checking RPATH system')
        check_rpath(self.webots_home)
        os.chdir(self.packaging_path)

        # create package folders
        print('creating folders')

        for folder in self.package_folders:
            self.make_dir(folder)

        # copy files in package
        print('copying files')
        for file in self.package_files:
            self.copy_file(file)
        os.chdir(self.packaging_path)

        # bundles usually have a 'Resources' folder with a capital 'R'
        os.rename(os.path.join(self.package_webots_path, 'Contents', 'resources'),
                  os.path.join(self.package_webots_path, 'Contents', 'Resources'))

        # create Qt symlinks
        path = os.path.join(self.package_webots_path, 'Contents', 'Frameworks')
        for file in os.listdir(path):
            module = file.split('.')[0]
            print(module)
            os.symlink(os.path.join('Versions', 'Current', module), os.path.join(path, file, module))
            os.symlink(os.path.join('Versions', 'Current', 'Headers'), os.path.join(path, file, 'Headers'))
            os.symlink(os.path.join('Versions', 'Current', 'Resources'), os.path.join(path, file, 'Resources'))
            os.symlink(os.path.join('A'), os.path.join(path, file, 'Versions', 'Current'))

        # sign the bundle
        os.system(f'codesign --deep -f -s - {self.package_webots_path}')

        data = {
            'title': 'Webots',
            'icon': os.path.join(self.webots_home, 'Contents', 'Resources', 'webots_icon.icns'),
            'icon-size': 72,
            'background': os.path.join(self.packaging_path, 'MacOSXBackground.png'),
            'format': 'UDBZ',
            'window': {
                'position': {
                    'x': 400, 'y': 100
                },
                'size': {
                    'width': 480, 'height': 580
                }
            },
            'contents': [
                {'x': 375, 'y': 100, 'type': 'link', 'path': '/Applications'},
                {'x': 100, 'y': 100, 'type': 'file', 'path': self.bundle_name}
            ],
            'code-sign': {
                'signing-identity': '-'
            }
        }
        with open(os.path.join(self.distribution_path, 'appdmg.json'), 'w') as f:
            f.write(json.dumps(data))
        print('generate DMG image')
        os.system(f"appdmg {self.distribution_path}/appdmg.json "
                  f"{self.distribution_path}/{self.application_name_lowercase_and_dashes}-{self.package_version}.dmg")

        # clear distribution folder
        remove_force(f"{self.distribution_path}/appdmg.json")

        print('Done.\n')

    def make_dir(self, directory):
        # create folder in distribution path
        if directory.startswith('Contents/'):
            destination_dir = os.path.join(self.package_webots_path, directory)
        else:
            destination_dir = os.path.join(self.package_webots_path, 'Contents', directory)
        if not os.path.isdir(destination_dir):
            os.makedirs(destination_dir)

    def copy_file(self, path):
        if path.startswith('Contents/'):
            destination_dir = os.path.join(self.package_webots_path, os.path.dirname(path))
        else:
            destination_dir = os.path.join(self.package_webots_path, 'Contents', os.path.dirname(path))
        source_path = os.path.join(self.webots_home, path)
        if not os.path.isfile(source_path):
            source_path = os.path.join(self.webots_home, 'Contents', path)
            if not os.path.isfile(source_path):
                sys.stderr.write(f'File not found: {source_path}\n')
                sys.exit(1)

        super().copy_file(path)

        # copy in distribution folder
        shutil.copy(source_path, destination_dir)

    def compute_name_with_prefix_and_extension(self, path, options):
        platform_independent = 'linux' not in options and 'windows' not in options and 'mac' not in options
        if sys.platform == 'darwin' and (platform_independent or 'mac' in options):
            if 'dll' in options:
                basename = os.path.basename(path)
                if basename.startswith('_'):
                    basename = basename + '.dylib'
                else:
                    basename = 'lib' + basename + '.dylib'
                return os.path.join(os.path.dirname(path), basename)
            return path
        return ""
