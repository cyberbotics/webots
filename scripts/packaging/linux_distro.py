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

from generic_distro import WebotsPackage, remove_force, symlink_force, print_error_message_and_exit
import distro  # needed to retrieve the Ubuntu version
import glob
import os
import shutil
import subprocess
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
        "libxcb-xinerama.so.0"
    ]
    USR_LIB_X68_64_20_04 = [
        "libHalf.so.24",
        "libIex-2_3.so.24",
        "libIexMath-2_3.so.24",
        "libIlmThread-2_3.so.24",
        "libIlmImf-2_3.so.24",
        "libwebp.so.6",
        "libzip.so.5",  # needed by Robotis OP2
        "libx264.so.155"
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

    def __init__(self, package_name):
        super().__init__(package_name)
        self.package_webots_path = os.path.join(self.distribution_path, 'debian', 'usr', 'local',
                                                self.application_name_lowercase_and_dashes)
        self.snap_script_path = os.path.join(self.packaging_path,  self.application_name_lowercase_and_dashes + '.snap')

        self.tarball_enabled = True
        self.deb_enabled = distro.version() == '20.04'
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

        if self.tarball_enabled or self.deb_enabled:
            self.add_ros_dependencies(self.package_webots_path, 'DEB')

            # copy OpenSSL libraries from Ubuntu 20.04 system and needed on Ubuntu 22.04
            system_lib_path = os.path.join('/usr', 'lib', 'x86_64-linux-gnu')
            package_webots_lib = os.path.join(self.package_webots_path, 'lib', 'webots')
            if distro.version() == '22.04':
                shutil.copy(os.path.join(self.webots_home, 'lib', 'webots', 'libcrypto.so.1.1'),
                            os.path.join(package_webots_lib, 'libcrypto.so.1.1'))
                shutil.copy(os.path.join(self.webots_home, 'lib', 'webots', 'libcrypto.so'),
                            os.path.join(package_webots_lib, 'libcrypto.so'))
                shutil.copy(os.path.join(self.webots_home, 'lib', 'webots', 'libssl.so.1.1'),
                            os.path.join(package_webots_lib, 'libssl.so.1.1'))
                shutil.copy(os.path.join(self.webots_home, 'lib', 'webots', 'libssl.so'),
                            os.path.join(package_webots_lib, 'libssl.so'))
            else:  # Ubuntu 20.04
                openssl_libs = ['libcrypto.so.1.1', 'libcrypto.so', 'libssl.so.1.1', 'libssl.so']
                for lib in openssl_libs:
                    shutil.copy(os.path.join(system_lib_path, lib), package_webots_lib)

        if self.deb_enabled:
            self.create_debian_bundle()
        if self.tarball_enabled:
            self.create_tarball_bundle()
        if self.snap_enabled:
            self.create_snap_bundle()

        remove_force(os.path.join(self.distribution_path, 'debian'))
        print('\nDone.\n')

    def create_debian_bundle(self):
        print("\ncreating the debian package")

        # copy webots application files needed by the debian package
        packaging_files = [['webots.mime', 'mime-info'],
                           ['webots.keys', 'mime-info'],
                           ['webots.png', 'pixmaps'],
                           ['webots_doc.png', 'pixmaps'],
                           ['webots.applications', 'application-registry'],
                           ['webots.desktop', 'applications'],
                           ['webots.desktop', 'app-install/desktop']]
        for pair in packaging_files:
            dst = os.path.join(self.distribution_path, 'debian', 'usr', 'share', pair[1])
            if not os.path.isdir(dst):
                os.makedirs(dst)
            shutil.copy(os.path.join(self.packaging_path, pair[0]), dst)

        # create symlink to webots binary needed by debian package
        os.makedirs(os.path.join(self.distribution_path, 'debian', 'usr', 'local', 'bin'))
        symlink_force(f"/usr/local/{self.application_name_lowercase_and_dashes}/webots",
                      os.path.join(self.distribution_path, 'debian', 'usr', 'local', 'bin', 'webots'))

        # add conflicting library not available in Ubuntu 22.04
        # so that the Robotis OP2 robot window works out of the box
        system_lib_path = os.path.join('/usr', 'lib', 'x86_64-linux-gnu')
        package_webots_lib = os.path.join(self.package_webots_path, 'lib', 'webots')
        if distro.version() == '22.04':
            shutil.copy(os.path.join(system_lib_path, 'libzip.so.4'), package_webots_lib)
        else:
            shutil.copy(os.path.join(system_lib_path, 'libzip.so.5'), package_webots_lib)

        # write 'DEBIAN/control' file required to create debian package
        os.makedirs(os.path.join(self.distribution_path, 'debian', 'DEBIAN'))
        # compute package size
        size_result = subprocess.run(["du", "-sx", os.path.join(self.distribution_path, 'debian')], stdout=subprocess.PIPE)
        with open(os.path.join(self.distribution_path, 'debian', 'DEBIAN', 'control'), 'w') as f:
            f.write(
                f"Package: {self.application_name_lowercase_and_dashes}\n"
                "Version: " + self.package_version[1:] + "\n"  # remove initial R from version not supported
                "Section: science\n"
                "Priority: optional\n"
                "Architecture: amd64\n"
                f"Installed-Size: {size_result.stdout.decode().split()[0]}\n"
                "Depends: make, g++, libatk1.0-0 (>= 1.9.0), ffmpeg, libdbus-1-3, libfreeimage3 (>= 3.15.4-3), "
                "libglib2.0-0 (>= 2.10.0), libegl1, libglu1-mesa | libglu1, libgtk-3-0, "
                "libnss3, libstdc++6 (>= 4.0.2-4), libxaw7, libxrandr2, libxrender1, "
                "libssh-dev, libzip-dev, xserver-xorg-core, libxslt1.1, "
                "libfreetype6, libxkbcommon-x11-0, libxcb-keysyms1, libxcb-image0, libxcb-icccm4, "
                "libxcb-randr0, libxcb-render-util0, libxcb-xinerama0\n"
                "Conflicts: webots-for-nao\n"
                "Maintainer: Olivier Michel <Olivier.Michel@cyberbotics.com>\n"
                "Description: Mobile robot simulation software\n"
                " Webots is a fast prototyping and simulation software\n"
                " which allows you to model, program and simulate any mobile\n"
                " robot, including wheeled, legged, swimming and flying robots.\n"
                " Transfer facilities allows you to transfer the robot\n"
                " controller from the simulation onto a real robot.\n"
            )

        os.chdir(self.distribution_path)
        subprocess.run(["fakeroot", "dpkg-deb", "-Zgzip", "--build", 'debian', self.distribution_path])
        os.chdir(self.packaging_path)

    def create_tarball_bundle(self):
        print("\ncreating the {}/{}-{}-x86-64.tar.bz2 tarball"
              .format(self.distribution_path, self.application_name_lowercase_and_dashes, self.package_version))

        # add specific libraries needed for tarball package
        usr_lib_x68_64 = self.USR_LIB_X68_64
        usr_lib_x68_64.append('libjpeg.so.8')
        if distro.version() == '22.04':
            usr_lib_x68_64 += self.USR_LIB_X68_64_22_04
            usr_lib_x68_64.append('libraw.so.20')
        else:
            usr_lib_x68_64 += self.USR_LIB_X68_64_20_04
            usr_lib_x68_64.append('libraw.so.19')
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

        usr_lib_x68_64_linux_gnu += self.USR_LIB_X68_64 + self.USR_LIB_X68_64_20_04

        for lib in usr_lib_x68_64_linux_gnu:
            self.snap_script.write("cp /usr/lib/x86_64-linux-gnu/{} $DESTDIR/usr/lib/x86_64-linux-gnu/\n".format(lib))
        self.snap_script.write("mkdir $DESTDIR/usr/share/webots/include/libssh\n")
        self.snap_script.write("cp -a /usr/include/libssh $DESTDIR/usr/share/webots/include/libssh/\n")
        self.snap_script.write("mkdir $DESTDIR/usr/share/webots/include/libzip\n")
        self.snap_script.write("cp -a /usr/include/zip.h $DESTDIR/usr/share/webots/include/libzip/\n")
        self.snap_script.write("cp /usr/include/zipconf.h $DESTDIR/usr/share/webots/include/libzip/\n")
        self.snap_script.write("cp $WEBOTS_HOME/scripts/packaging/webots_snap.desktop "
                               "$DESTDIR/usr/share/webots/resources/webots.desktop\n")
        self.add_ros_dependencies("$DESTDIR/usr/share/webots", 'SNAP')
        self.snap_script.close()
        self.set_execution_rights(self.snap_script_path)

    def make_dir(self, directory):
        if self.tarball_enabled or self.deb_enabled:
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

        if self.tarball_enabled or self.deb_enabled:
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

    def add_ros_dependencies(self, path, mode):
        if sys.platform != 'linux' or distro.version() != '20.04':
            return
        noetic_libs = ['libcontroller_manager.so',
                       'libclass_loader.so',
                       'libroscpp.so',
                       'librosconsole.so',
                       'libroscpp_serialization.so',
                       'libroslib.so',
                       'librostime.so',
                       'libxmlrpcpp.so',
                       'libcpp_common.so',
                       'librosconsole_log4cxx.so',
                       'librosconsole_backend_interface.so',
                       'librosconsole_backend_interface.so']
        system_libs = ['libboost_thread.so.1.71.0',
                       'libboost_chrono.so.1.71.0',
                       'libboost_filesystem.so.1.71.0',
                       'liblog4cxx.so.10',
                       'libboost_regex.so.1.71.0',
                       'libconsole_bridge.so.0.4',
                       'libapr-1.so.0',
                       'libaprutil-1.so.0',
                       'libboost_program_options.so.1.71.0',
                       'libpython3.8.so.1.0']

        ros_lib_path = os.path.join(path, 'projects', 'default', 'controllers', 'ros', 'lib', 'ros')
        if mode == 'SNAP':
            # add instructions to copy ros libraries to snap script
            self.snap_script.write(f"mkdir -p {ros_lib_path}\n")
            for lib in noetic_libs:
                self.snap_script.write(f"cp /opt/ros/noetic/lib/{lib} {ros_lib_path}\n")
            for lib in system_libs:
                self.snap_script.write(f"cp /usr/lib/x86_64-linux-gnu/{lib} {ros_lib_path}\n")
        else:
            # copy ros libraries in distribution folder
            os.makedirs(ros_lib_path)
            for lib in noetic_libs:
                shutil.copy(f"/opt/ros/noetic/lib/{lib}", ros_lib_path)
            for lib in system_libs:
                shutil.copy(f"/usr/lib/x86_64-linux-gnu/{lib}", ros_lib_path)
