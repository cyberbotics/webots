"""This script creates the skeletton of a catkin package."""

from __future__ import print_function

import argparse
import os
import sys

from catkin_pkg.package_templates import create_package_files, PackageTemplate


def main(argv=sys.argv[1:], parent_path=os.getcwd()):
    parser = argparse.ArgumentParser(
        description='Creates a new catkin package')
    parser.add_argument('name',
                        nargs=1,
                        help='The name for the package')
    parser.add_argument('--meta',
                        action='store_true',
                        help='Creates meta-package files')
    parser.add_argument('dependencies',
                        nargs='*',
                        help='Catkin package Dependencies')
    parser.add_argument('-s', '--sys-deps',
                        nargs='*',
                        help='System Dependencies')
    parser.add_argument('-b', '--boost-comps',
                        nargs='*',
                        help='Boost Components')
    parser.add_argument('-V', '--pkg_version',
                        action='store',
                        help='Initial Package version')
    parser.add_argument('-D', '--description',
                        action='store',
                        help='Description')
    parser.add_argument('-l', '--license',
                        action='append',
                        help='Name for License, (e.g. BSD, MIT, GPLv3...)')
    parser.add_argument('-a', '--author',
                        action='append',
                        help='A single author, may be used multiple times')
    parser.add_argument('-m', '--maintainer',
                        action='append',
                        help='A single maintainer, may be used multiple times')
    rosdistro_name = os.environ['ROS_DISTRO'] if 'ROS_DISTRO' in os.environ else None
    parser.add_argument('--rosdistro', required=rosdistro_name is None, default=rosdistro_name, help='The ROS distro (default: environment variable ROS_DISTRO if defined)')

    args = parser.parse_args(argv)

    try:
        package_name = args.name[0]
        target_path = os.path.join(parent_path, package_name)
        package_template = PackageTemplate._create_package_template(
            package_name=package_name,
            description=args.description,
            licenses=args.license or [],
            maintainer_names=args.maintainer,
            author_names=args.author,
            version=args.pkg_version,
            catkin_deps=args.dependencies,
            system_deps=args.sys_deps,
            boost_comps=args.boost_comps)
        create_package_files(target_path=target_path,
                             package_template=package_template,
                             rosdistro=args.rosdistro,
                             newfiles={},
                             meta=args.meta)
        print('Successfully created files in %s. Please adjust the values in package.xml.' % target_path)
    except ValueError as vae:
        parser.error(str(vae))
