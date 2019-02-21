# Software License Agreement (BSD License)
#
# Copyright (c) 2012, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

from __future__ import print_function

import getpass
import os
import string
import sys

from catkin_pkg.cmake import configure_file
from catkin_pkg.cmake import get_metapackage_cmake_template_path
from catkin_pkg.package import Dependency
from catkin_pkg.package import Package
from catkin_pkg.package import PACKAGE_MANIFEST_FILENAME
from catkin_pkg.package import Person


class PackageTemplate(Package):

    def __init__(self, catkin_deps=None, system_deps=None, boost_comps=None, **kwargs):
        super(PackageTemplate, self).__init__(**kwargs)
        self.catkin_deps = catkin_deps or []
        self.system_deps = system_deps or []
        self.boost_comps = boost_comps or []
        self.validate()

    @staticmethod
    def _create_package_template(package_name, description=None, licenses=None,
                                 maintainer_names=None, author_names=None,
                                 version=None, catkin_deps=None, system_deps=None,
                                 boost_comps=None):
        """
        Alternative factory method mapping CLI args to argument for Package class.

        :param package_name:
        :param description:
        :param licenses:
        :param maintainer_names:
        :param authors:
        :param version:
        :param catkin_deps:
        """
        # Sort so they are alphebetical
        licenses = list(licenses or ['TODO'])
        licenses.sort()
        if not maintainer_names:
            maintainer_names = [getpass.getuser()]
        maintainer_names = list(maintainer_names or [])
        maintainer_names.sort()
        maintainers = []
        for maintainer_name in maintainer_names:
            maintainers.append(
                Person(maintainer_name,
                       '%s@todo.todo' % maintainer_name.split()[-1])
            )
        author_names = list(author_names or [])
        author_names.sort()
        authors = []
        for author_name in author_names:
            authors.append(Person(author_name))
        catkin_deps = list(catkin_deps or [])
        catkin_deps.sort()
        pkg_catkin_deps = []
        depends = []
        build_depends = []
        exec_depends = []
        buildtool_depends = [Dependency('catkin')]
        for dep in catkin_deps:
            if dep.lower() == 'catkin':
                catkin_deps.remove(dep)
                continue
            if dep.lower() == 'genmsg':
                sys.stderr.write('WARNING: Packages with messages or services should not depend on genmsg, but on message_generation and message_runtime\n')
                buildtool_depends.append(Dependency('genmsg'))
                continue
            if dep.lower() == 'message_generation':
                if 'message_runtime' not in catkin_deps:
                    sys.stderr.write('WARNING: Packages with messages or services should depend on both message_generation and message_runtime\n')
                build_depends.append(Dependency('message_generation'))
                continue
            if dep.lower() == 'message_runtime':
                if 'message_generation' not in catkin_deps:
                    sys.stderr.write('WARNING: Packages with messages or services should depend on both message_generation and message_runtime\n')
                exec_depends.append(Dependency('message_runtime'))
                continue
            pkg_catkin_deps.append(Dependency(dep))
        for dep in pkg_catkin_deps:
            depends.append(dep)
        if boost_comps:
            if not system_deps:
                system_deps = ['boost']
            elif 'boost' not in system_deps:
                system_deps.append('boost')
        for dep in system_deps or []:
            if not dep.lower().startswith('python-'):
                depends.append(Dependency(dep))
            else:
                exec_depends.append(Dependency(dep))
        package_temp = PackageTemplate(
            name=package_name,
            version=version or '0.0.0',
            description=description or 'The %s package' % package_name,
            buildtool_depends=buildtool_depends,
            build_depends=build_depends,
            depends=depends,
            exec_depends=exec_depends,
            catkin_deps=catkin_deps,
            system_deps=system_deps,
            boost_comps=boost_comps,
            licenses=licenses,
            authors=authors,
            maintainers=maintainers,
            urls=[])
        return package_temp


def read_template_file(filename, rosdistro):
    template_dir = os.path.join(os.path.dirname(__file__), 'templates')
    templates = []
    templates.append(os.path.join(template_dir, rosdistro, '%s.in' % filename))
    templates.append(os.path.join(template_dir, '%s.in' % filename))
    for template in templates:
        if os.path.isfile(template):
            with open(template, 'r') as fhand:
                template_contents = fhand.read()
            return template_contents
    raise IOError(
        'Could not read template for ROS distro '
        "'{}' at '{}': ".format(rosdistro, ', '.join(templates)) +
        'no such file or directory'
    )


def _safe_write_files(newfiles, target_dir):
    """
    Write file contents to target_dir/filepath for all entries of newfiles.

    Aborts early if files exist in places for new files or directories

    :param newfiles: a dict {filepath: contents}
    :param target_dir: a string
    """
    # first check no filename conflict exists
    for filename in newfiles:
        target_file = os.path.join(target_dir, filename)
        if os.path.exists(target_file):
            raise ValueError('File exists: %s' % target_file)
        dirname = os.path.dirname(target_file)
        while dirname != target_dir:
            if os.path.isfile(dirname):
                raise ValueError('Cannot create directory, file exists: %s' %
                                 dirname)
            dirname = os.path.dirname(dirname)

    for filename, content in newfiles.items():
        target_file = os.path.join(target_dir, filename)
        dirname = os.path.dirname(target_file)
        if not os.path.exists(dirname):
            os.makedirs(dirname)
        # print(target_file, content)
        with open(target_file, 'ab') as fhand:
            fhand.write(content.encode())
        print('Created file %s' % os.path.relpath(target_file, os.path.dirname(target_dir)))


def create_package_files(target_path, package_template, rosdistro,
                         newfiles=None, meta=False):
    """
    Create several files from templates to start a new package.

    :param target_path: parent folder where to create the package
    :param package_template: contains the required information
    :param rosdistro: name of the distro to look up respective template
    :param newfiles: dict {filepath: contents} for additional files to write
    """
    if newfiles is None:
        newfiles = {}
    # allow to replace default templates when path string is equal
    manifest_path = os.path.join(target_path, PACKAGE_MANIFEST_FILENAME)
    if manifest_path not in newfiles:
        newfiles[manifest_path] = \
            create_package_xml(package_template, rosdistro, meta=meta)
    cmake_path = os.path.join(target_path, 'CMakeLists.txt')
    if cmake_path not in newfiles:
        newfiles[cmake_path] = create_cmakelists(package_template, rosdistro, meta=meta)
    _safe_write_files(newfiles, target_path)
    if 'roscpp' in package_template.catkin_deps:
        fname = os.path.join(target_path, 'include', package_template.name)
        os.makedirs(fname)
        print('Created folder %s' % os.path.relpath(fname, os.path.dirname(target_path)))
    if 'roscpp' in package_template.catkin_deps or \
            'rospy' in package_template.catkin_deps:
        fname = os.path.join(target_path, 'src')
        os.makedirs(fname)
        print('Created folder %s' % os.path.relpath(fname, os.path.dirname(target_path)))


class CatkinTemplate(string.Template):
    """subclass to use @ instead of $ as markers."""

    delimiter = '@'
    escape = '@'


def create_cmakelists(package_template, rosdistro, meta=False):
    """Create CMake file contents from the template.

    :param package_template: contains the required information
    :returns: file contents as string
    """
    if meta:
        template_path = get_metapackage_cmake_template_path()
        temp_dict = {
            'name': package_template.name,
            'metapackage_arguments': '',
        }
        return configure_file(template_path, temp_dict)
    else:
        cmakelists_txt_template = read_template_file('CMakeLists.txt', rosdistro)
        ctemp = CatkinTemplate(cmakelists_txt_template)
        if package_template.catkin_deps == []:
            components = ''
        else:
            components = ' COMPONENTS\n  %s\n' % '\n  '.join(package_template.catkin_deps)
        boost_find_package = \
            ('' if not package_template.boost_comps
             else ('find_package(Boost REQUIRED COMPONENTS %s)\n' %
                   ' '.join(package_template.boost_comps)))
        system_find_package = ''
        for sysdep in package_template.system_deps:
            if sysdep == 'boost':
                continue
            if sysdep.startswith('python-'):
                system_find_package += '# '
            system_find_package += 'find_package(%s REQUIRED)\n' % sysdep
        # provide dummy values
        catkin_depends = (' '.join(package_template.catkin_deps)
                          if package_template.catkin_deps
                          else 'other_catkin_pkg')
        system_depends = (' '.join(package_template.system_deps)
                          if package_template.system_deps
                          else 'system_lib')
        message_pkgs = [pkg for pkg in package_template.catkin_deps if pkg.endswith('_msgs')]
        if message_pkgs:
            message_depends = '#   %s' % '#   '.join(message_pkgs)
        else:
            message_depends = '#   std_msgs  # Or other packages containing msgs'
        temp_dict = {'name': package_template.name,
                     'components': components,
                     'include_directories': _create_include_macro(package_template),
                     'boost_find': boost_find_package,
                     'systems_find': system_find_package,
                     'catkin_depends': catkin_depends,
                     'system_depends': system_depends,
                     'target_libraries': _create_targetlib_args(package_template),
                     'message_dependencies': message_depends
                     }
        return ctemp.substitute(temp_dict)


def _create_targetlib_args(package_template):
    result = '#   ${catkin_LIBRARIES}\n'
    if package_template.boost_comps:
        result += '#   ${Boost_LIBRARIES}\n'
    if package_template.system_deps:
        result += (''.join(['#   ${%s_LIBRARIES}\n' %
                            sdep for sdep in package_template.system_deps]))
    return result


def _create_include_macro(package_template):
    includes = ['# include']
    includes.append(('  ' if package_template.catkin_deps else '# ') + '${catkin_INCLUDE_DIRS}')
    if package_template.boost_comps:
        includes.append('  ${Boost_INCLUDE_DIRS}')
    if package_template.system_deps:
        deplist = []
        for sysdep in package_template.system_deps:
            if not sysdep.startswith('python-'):
                deplist.append(sysdep)
        if deplist:
            todo_incl = '# TODO: Check names of system library include directories'
            includes.append(todo_incl + (' (%s)' % ', '.join(deplist)))
            includes.extend(['  ${%s_INCLUDE_DIRS}' % sysdep for sysdep in deplist])
    result = ''
    if includes:
        result += '\n'.join(includes)
    return result


def _create_depend_tag(dep_type,
                       name,
                       version_eq=None,
                       version_lt=None,
                       version_lte=None,
                       version_gt=None,
                       version_gte=None):
    """Create xml snippet for package.xml."""
    version_string = []
    for key, var in {'version_eq': version_eq,
                     'version_lt': version_lt,
                     'version_lte': version_lte,
                     'version_gt': version_gt,
                     'version_gte': version_gte}.items():
        if var is not None:
            version_string.append(' %s="%s"' % (key, var))
    result = '  <%s%s>%s</%s>\n' % (dep_type,
                                    ''.join(version_string),
                                    name,
                                    dep_type)
    return result


def create_package_xml(package_template, rosdistro, meta=False):
    """
    Create package xml file content.

    :param package_template: contains the required information
    :returns: file contents as string
    """
    package_xml_template = \
        read_template_file(PACKAGE_MANIFEST_FILENAME, rosdistro)
    ctemp = CatkinTemplate(package_xml_template)
    temp_dict = {}
    for key in package_template.__slots__:
        temp_dict[key] = getattr(package_template, key)

    if package_template.version_compatibility:
        temp_dict['version_compatibility'] = \
            ' compatibility="%s"' % package_template.version_compatibility
    else:
        temp_dict['version_compatibility'] = ''

    if not package_template.description:
        temp_dict['description'] = 'The %s package ...' % package_template.name

    licenses = []
    for plicense in package_template.licenses:
        licenses.append('  <license>%s</license>\n' % plicense)
    temp_dict['licenses'] = ''.join(licenses)

    def get_person_tag(tagname, person):
        email_string = (
            '' if person.email is None else 'email="%s"' % person.email
        )
        return '  <%s %s>%s</%s>\n' % (tagname, email_string,
                                       person.name, tagname)

    maintainers = []
    for maintainer in package_template.maintainers:
        maintainers.append(get_person_tag('maintainer', maintainer))
    temp_dict['maintainers'] = ''.join(maintainers)

    urls = []
    for url in package_template.urls:
        type_string = ('' if url.type is None
                       else 'type="%s"' % url.type)
        urls.append('    <url %s >%s</url>\n' % (type_string, url.url))
    temp_dict['urls'] = ''.join(urls)

    authors = []
    for author in package_template.authors:
        authors.append(get_person_tag('author', author))
    temp_dict['authors'] = ''.join(authors)

    dependencies = []
    dep_map = {
        'build_depend': package_template.build_depends,
        'build_export_depend': package_template.build_export_depends,
        'buildtool_depend': package_template.buildtool_depends,
        'exec_depend': package_template.exec_depends,
        'test_depend': package_template.test_depends,
        'conflict': package_template.conflicts,
        'replace': package_template.replaces
    }
    for dep_type in ['buildtool_depend', 'build_depend', 'build_export_depend',
                     'exec_depend', 'test_depend', 'conflict', 'replace']:
        for dep in sorted(dep_map[dep_type], key=lambda x: x.name):
            if 'depend' in dep_type:
                dep_tag = _create_depend_tag(
                    dep_type,
                    dep.name,
                    dep.version_eq,
                    dep.version_lt,
                    dep.version_lte,
                    dep.version_gt,
                    dep.version_gte
                )
                dependencies.append(dep_tag)
            else:
                dependencies.append(_create_depend_tag(dep_type,
                                                       dep.name))
    temp_dict['dependencies'] = ''.join(dependencies)

    exports = []
    if package_template.exports is not None:
        for export in package_template.exports:
            if export.content is not None:
                print('WARNING: Create package does not know how to '
                      'serialize exports with content: '
                      '%s, %s, ' % (export.tagname, export.attributes) +
                      '%s' % (export.content),
                      file=sys.stderr)
            else:
                attribs = [' %s="%s"' % (k, v) for (k, v) in export.attributes.items()]
                line = '    <%s%s/>\n' % (export.tagname, ''.join(attribs))
                exports.append(line)

    if meta:
        exports.append('    <metapackage/>')
    temp_dict['exports'] = ''.join(exports)

    temp_dict['components'] = package_template.catkin_deps

    return ctemp.substitute(temp_dict)
