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

"""Library for providing the relevant information from the package manifest for the Python setup.py file."""

from __future__ import print_function

import os
import sys

from .package import InvalidPackage, parse_package


def generate_distutils_setup(package_xml_path=os.path.curdir, **kwargs):
    """
    Extract the information relevant for distutils from the package manifest.

    The following keys will be set:

    The "name" and "version" are taken from the eponymous tags.

    A single maintainer will set the keys "maintainer" and
    "maintainer_email" while multiple maintainers are merged into the
    "maintainer" fields (including their emails). Authors are handled
    likewise.

    The first URL of type "website" (or without a type) is used for
    the "url" field.

    The "description" is taken from the eponymous tag if it does not
    exceed 200 characters. If it does "description" contains the
    truncated text while "description_long" contains the complete.

    All licenses are merged into the "license" field.

    :param kwargs: All keyword arguments are passed through. The above
        mentioned keys are verified to be identical if passed as a
        keyword argument

    :returns: return dict populated with parsed fields and passed
        keyword arguments
    :raises: :exc:`InvalidPackage`
    :raises: :exc:`IOError`
    """
    package = parse_package(package_xml_path)

    data = {}
    data['name'] = package.name
    data['version'] = package.version

    # either set one author with one email or join all in a single field
    if len(package.authors) == 1 and package.authors[0].email is not None:
        data['author'] = package.authors[0].name
        data['author_email'] = package.authors[0].email
    else:
        data['author'] = ', '.join([('%s <%s>' % (a.name, a.email) if a.email is not None else a.name) for a in package.authors])

    # either set one maintainer with one email or join all in a single field
    if len(package.maintainers) == 1:
        data['maintainer'] = package.maintainers[0].name
        data['maintainer_email'] = package.maintainers[0].email
    else:
        data['maintainer'] = ', '.join(['%s <%s>' % (m.name, m.email) for m in package.maintainers])

    # either set the first URL with the type 'website' or the first URL of any type
    websites = [url.url for url in package.urls if url.type == 'website']
    if websites:
        data['url'] = websites[0]
    elif package.urls:
        data['url'] = package.urls[0].url

    if len(package.description) <= 200:
        data['description'] = package.description
    else:
        data['description'] = package.description[:197] + '...'
        data['long_description'] = package.description

    data['license'] = ', '.join(package.licenses)

    # pass keyword arguments and verify equality if generated and passed in
    for k, v in kwargs.items():
        if k in data:
            if v != data[k]:
                raise InvalidPackage('The keyword argument "%s" does not match the information from package.xml: "%s" != "%s"' % (k, v, data[k]), package_xml_path)
        else:
            data[k] = v

    return data


def get_global_bin_destination():
    return 'bin'


def get_global_etc_destination():
    return 'etc'


def get_global_include_destination():
    return 'include'


def get_global_lib_destination():
    return 'lib'


def get_global_libexec_destination():
    return 'lib'


def get_global_python_destination():
    dest = 'lib/python%u.%u/' % (sys.version_info[0], sys.version_info[1])
    if '--install-layout=deb' not in sys.argv[1:]:
        dest += 'site-packages'
    else:
        dest += 'dist-packages'
    return dest


def get_global_share_destination():
    return 'share'


def get_package_bin_destination(pkgname):
    return os.path.join(get_global_libexec_destination(), pkgname)


def get_package_etc_destination(pkgname):
    return os.path.join(get_global_etc_destination(), pkgname)


def get_package_include_destination(pkgname):
    return os.path.join(get_global_include_destination(), pkgname)


def get_package_lib_destination(_pkgname):
    return get_global_lib_destination()


def get_package_python_destination(pkgname):
    return os.path.join(get_global_python_destination(), pkgname)


def get_package_share_destination(pkgname):
    return os.path.join(get_global_share_destination(), pkgname)
