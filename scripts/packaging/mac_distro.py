#!/usr/bin/env python3.10

# Copyright 1996-2022 Cyberbotics Ltd.
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

"""Generate macOS Webots package."""

from generic_distro import WebotsPackage
import json
import os
import shutil
import sys


class MacWebotsPackage(WebotsPackage):
    def __init__(self, application_name_lowercase_and_dashes):
        self.application_file_path = application_name_lowercase_and_dashes + '.mac'
        self.bundle_name = self.application_name + '.app'
        self.package_webots_path = os.path.join(self.distribution_path, self.bundle_name)

        # remove previous distribution package
        path = os.path.join(self.distribution, self.bundle_name)
        if os.path.exists(path):
            modules = ['Contents', 'webots', 'docs', 'bin', 'include', 'lib', 'projects', 'resources', 'transfer', 'util']
            for name in modules:
                self.remove_force(os.join(path, name))
        self.remove_force(os.path.join(self.distribution,
                                       f"{self.application_name_lowercase_and_dashes}-{self.package_version}.dmg"))
        os.makedirs(path)

    def create_webots_bundle(self):
        super().create_webots_bundle(self)

        # create package folders
        print('create folders')
        for folder in self.package_folders:
            self.make_dir(folder)

        # copy files in package
        print('copy files')
        for file in self.package_files:
            self.copy_file(file)

        frameworks_path = os.path.join(self.package_webot_path, 'lib', 'webots', 'Contents', 'Frameworks')
        qt_modules = ['QtConcurrent', 'QtCore', 'QtDBus', 'QtGui', 'QtNetwork', 'QtOpenGL', 'QtOpenGLWidgets', 'QtPrintSupport',
                      'QtQml', 'QtWebSockets', 'QtWidgets', 'QtXml']
        for name in qt_modules:
            module_path = os.path.join(frameworks_path, name + '.framework')
            self.symlink_force(f"{module_path}Versions/A/{name}", f"{module_path}/name")
            self.symlink_force(f"{module_path}/Versions/A/Headers", f"{module_path}/Headers")  # TODO check directory
            self.symlink_force(f"{module_path}/A", f"{module_path}/Versions/Current")

        data = {
            'title': 'Webots',
            "icon": os.path.join(self.webots_home, "Contents", "Resources", 'webots_icon.icns'),
            "icon-size": 72,
            "background": os.path.join(self.packaging_path, "MacOSXBackground.png"),
            "format": "UDBZ",
            "window": {
                "position": {
                    "x": 400, "y": 100
                },
                "size": {
                    "width": 480, "height": 580
                }
            },
            "contents": [
                {"x": 375, "y": 100, "type": "link", "path": "/Applications"},
                {"x": 100, "y": 100, "type": "file", "path": self.bundle_name}
            ]
        }
        with open(os.path.join(self.distribution_path, 'appdmg.json')) as f:
            f.write(json.dumps(data))
        os.system(f"appdmg {self.distribution_path}/appdmg.json "
                  "{self.distribution_path}/{self.application_name_lowercase_and_dashes}-{self.package_version}.dmg")

        # clear distribution folder
        # self.remove_force('appdmg.json') TODO

        print('Done.\n')

    def make_dir(self, directory):
        # TODO rel_dir_path = os.path.join('usr', 'local', self.application_name_lowercase_and_dashes,
        #                            os.path.relpath(directory, self.webots_home))

        # create folder in distribution path
        dst_dir = os.path.join(self.distribution_path, self.bundle_name, directory)
        if not os.path.isdir(dst_dir):
            os.makedirs(dst_dir)

    def copy_file(self, path):
        super().copy_file(path)

        dir_path = os.path.dirname(path)
        file_name = os.path.basename(path)

        # copy in distribution folder
        rel_dir_path = os.path.relpath(dir_path, self.webots_home)
        dst_dir = os.path.join(self.package_webots_path, rel_dir_path)
        shutil.copy(os.path.join(dir_path, file_name), dst_dir)

    def compute_name_with_prefix_and_extension(self, basename, options):
        platform_independent = 'linux' not in options and 'windows' not in options and 'mac' not in options
        if sys.platform == 'darwin' and (platform_independent or 'mac' in options):
            if 'dll' in options:
                return 'lib' + basename + '.dylib'
            return basename
        return ""
