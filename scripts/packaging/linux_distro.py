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

"""Generate Linux Webots tarball, Debian and snap packages."""

from generic_distro import WebotsPackage, remove_force, print_error_message_and_exit
import distro  # needed to retrieve the Ubuntu version
import glob
import os
import shutil
import sys
import tarfile


class LinuxWebotsPackage(WebotsPackage):
    USR_LIB_X68_64 = [
        "libfontconfig.so.1",
        "libfreetype.so.6",
        "libgomp.so.1",
        "liblcms2.so.2",
        "libopenjp2.so.7",
        "libpng16.so.16",
        "libssh-gcrypt.so.4",   # needed by Robotis OP2
        "libwebpmux.so.3",
        "libXi.so.6",
        "libXrender.so.1",
        "libxslt.so.1",
        "libxcb-keysyms.so.1",
        "libxcb-image.so.0",
        "libxcb-icccm.so.4",
        "libxcb-randr.so.0",
        "libxcb-render-util.so.0",
        "libxcb-xinerama.so.0",
        "libxcb-cursor.so.0"
    ]
    USR_LIB_X68_64_22_04 = [
        "libHalf-2_5.so.25",
        "libIex-2_5.so.25",
        "libIexMath-2_5.so.25",
        "libIlmThread-2_5.so.25",
        "libIlmImf-2_5.so.25",
        "libwebp.so.7",
        "libzip.so.4",  # needed by Robotis OP2
        "libx264.so.163"
    ]
    USR_LIB_X68_64_24_04 = [
        "libIex-3_1.so.30",
        "libIlmThread-3_1.so.30",
        "libwebp.so.7",
        "libx264.so.164",
        "libzip.so.4",
        "libHalf-3_1.so.30",
        "libIexMath-3_1.so.30",
        "libIlmImf-3_1.so.30"
    ]

    def __init__(self, package_name):
        super().__init__(package_name)
        self.package_webots_path = os.path.join(self.distribution_path, 'debian', 'usr', 'local',
                                                self.application_name_lowercase_and_dashes)
        self.snap_script_path = os.path.join(self.packaging_path,  self.application_name_lowercase_and_dashes + '.snap')

        self.tarball_enabled = True
        self.snap_enabled = True
        if self.snap_enabled:
            # open snap script file and write header
            try:
                self.snap_script = open(self.snap_script_path, 'w+')
                self.snap_script.write("#!/bin/bash\n"
                                       "# run this auto-generated script to install the "
                                       f"{self.application_name_lowercase_and_dashes} snap in \"$DESTDIR\"\n\n"
                                       "mkdir -p $DESTDIR\n"
                                       "mkdir -p $DESTDIR/lib\n"
                                       "mkdir -p $DESTDIR/lib/x86_64-linux-gnu\n"
                                       "mkdir -p $DESTDIR/usr\n"
                                       "mkdir -p $DESTDIR/usr/share\n"
                                       "mkdir -p $DESTDIR/usr/bin\n"
                                       "mkdir -p $DESTDIR/usr/lib\n"
                                       "mkdir -p $DESTDIR/usr/lib/x86_64-linux-gnu\n"
                                       f"mkdir $DESTDIR/usr/share/{self.application_name_lowercase_and_dashes}\n")
            except IOError:
                print_error_message_and_exit(f"Could not open file: {self.snap_script_path}.")

        # clear distribution package folder
        print('clear distribution folder')
        remove_force(os.path.join(self.distribution_path, 'debian'))
        for f in glob.glob(os.path.join(self.distribution_path,
                                        f"{self.application_name_lowercase_and_dashes}-{self.package_version}_*.deb")):
            remove_force(f)
        os.makedirs(self.package_webots_path)

    def create_webots_bundle(self, include_commit_file):
        super().create_webots_bundle(include_commit_file)

        # create package folders
        print('creating folders')
        for folder in self.package_folders:
            self.make_dir(folder)

        # copy files in package
        print('copying files')
        for file in self.package_files:
            self.copy_file(file)

        if self.tarball_enabled:
            # copy OpenSSL libraries from Ubuntu 20.04 system and needed on Ubuntu 22.04
            system_lib_path = os.path.join('/usr', 'lib', 'x86_64-linux-gnu')
            package_webots_lib = os.path.join(self.package_webots_path, 'lib', 'webots')
            if distro.version() == '22.04':  # Ubuntu 22.04 and 24.04
                openssl_libs = ['libcrypto.so.3', 'libcrypto.so', 'libssl.so.3', 'libssl.so']
                for lib in openssl_libs:
                    shutil.copy(os.path.join(system_lib_path, lib), package_webots_lib)

        if self.tarball_enabled:
            self.create_tarball_bundle()
        if self.snap_enabled:
            self.create_snap_bundle()

        remove_force(os.path.join(self.distribution_path, 'debian'))
        print('\nDone.\n')

    def create_tarball_bundle(self):
        print("\ncreating the {}/{}-{}-x86-64.tar.bz2 tarball"
              .format(self.distribution_path, self.application_name_lowercase_and_dashes, self.package_version))

        # add specific libraries needed for tarball package (Ubuntu 24.04)
        usr_lib_x68_64 = self.USR_LIB_X68_64
        usr_lib_x68_64.append('libjpeg.so.8')
        if distro.version() == '22.04':
            usr_lib_x68_64 += self.USR_LIB_X68_64_22_04
            usr_lib_x68_64.append('libraw.so.20')
        if distro.version() == '24.04':
            usr_lib_x68_64 += self.USR_LIB_X68_64_24_04
            usr_lib_x68_64.append('libraw.so.23')
        system_lib_path = os.path.join('/usr', 'lib', 'x86_64-linux-gnu')
        package_webots_lib = os.path.join(self.package_webots_path, 'lib', 'webots')
        for lib in usr_lib_x68_64:
            shutil.copy(os.path.join(system_lib_path, lib), package_webots_lib)

        os.chdir(self.package_webots_path)
        os.chdir('..')
        with tarfile.open(os.path.join(self.distribution_path,
                          f"{self.application_name_lowercase_and_dashes}-{self.package_version}-x86-64.tar.bz2"),
                          'w:bz2') as tar:
            tar.add(self.application_name_lowercase_and_dashes)
        os.chdir(self.packaging_path)

    def create_snap_bundle(self):
        print('\ncreating the snap package')
        # copy system libraries and include files
        # libraries specific to snap package
        usr_lib_x68_64_linux_gnu = ["libraw1394.so.11",
                                    "libPocoFoundation.so.62",
                                    "libcanberra-gtk-module",
                                    "libcanberra-gtk3-module"]

        for lib in usr_lib_x68_64_linux_gnu:
            self.snap_script.write("cp /usr/lib/x86_64-linux-gnu/{} $DESTDIR/usr/lib/x86_64-linux-gnu/\n".format(lib))
        self.snap_script.write("mkdir $DESTDIR/usr/share/webots/include/libssh\n")
        self.snap_script.write("cp -a /usr/include/libssh $DESTDIR/usr/share/webots/include/libssh/\n")
        self.snap_script.write("mkdir $DESTDIR/usr/share/webots/include/libzip\n")
        self.snap_script.write("cp -a /usr/include/zip.h $DESTDIR/usr/share/webots/include/libzip/\n")
        self.snap_script.write("cp /usr/include/zipconf.h $DESTDIR/usr/share/webots/include/libzip/\n")
        self.snap_script.write("cp $WEBOTS_HOME/scripts/packaging/webots_snap.desktop "
                               "$DESTDIR/usr/share/webots/resources/webots.desktop\n")
        self.snap_script.close()
        self.set_execution_rights(self.snap_script_path)

    def make_dir(self, directory):
        if self.tarball_enabled:
            # create folder in distribution path
            rel_dir_path = os.path.join('usr', 'local', self.application_name_lowercase_and_dashes, directory)
            dst_dir = os.path.join(self.distribution_path, 'debian', rel_dir_path)
            if not os.path.isdir(dst_dir):
                os.makedirs(dst_dir)

        if self.snap_enabled:
            # add mkdir instruction in snap script
            rel_dir_path = os.path.join('usr', 'share', self.application_name_lowercase_and_dashes, directory)
            self.snap_script.write("mkdir " + os.path.join('$DESTDIR', rel_dir_path) + "\n")

    def copy_file(self, path):
        super().copy_file(path)

        dir_path = os.path.dirname(path)
        file_name = os.path.basename(path)

        if self.tarball_enabled:
            # copy in distribution folder
            dst_dir = os.path.join(self.package_webots_path, dir_path)
            shutil.copy(os.path.join(self.webots_home, path), dst_dir)

        if self.snap_enabled:
            # add copy instruction in snap script
            protected_filename = file_name.replace('$', '\\$').replace('(', '\\(').replace(')', '\\)').replace(' ', '\\ ')
            self.snap_script.write(f"cp -a $WEBOTS_HOME/{os.path.join(dir_path, protected_filename)} "
                                   f"$DESTDIR/usr/share/{self.application_name_lowercase_and_dashes}/{dir_path}\n")

    def compute_name_with_prefix_and_extension(self, path, options):
        platform_independent = 'linux' not in options and 'windows' not in options and 'mac' not in options
        if sys.platform == 'linux' and (platform_independent or 'linux' in options):
            if 'dll' in options:
                basename = os.path.basename(path)
                if basename.startswith('_'):
                    basename = basename + '.so'
                else:
                    basename = 'lib' + basename + '.so'
                return os.path.join(os.path.dirname(path), basename)
            return path
        return ""
