#!/usr/bin/env python
# Copyright (c) 2009, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Willow Garage, Inc. nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# Author Tully Foote/tfoote@willowgarage.com, Ken Conley/kwc@willowgarage.com

"""
Library for detecting the current OS, including detecting specific
Linux distributions.
"""

from __future__ import print_function

import codecs
import locale
import os
import platform
import subprocess


def _read_stdout(cmd):
    try:
        pop = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        (std_out, std_err) = pop.communicate()
        # Python 2.6 compatibility
        if isinstance(std_out, str):
            return std_out.strip()
        return std_out.decode(encoding='UTF-8').strip()
    except:
        return None


def uname_get_machine():
    """
    Linux: wrapper around uname to determine if OS is 64-bit
    """
    return _read_stdout(['uname', '-m'])


def read_issue(filename="/etc/issue"):
    """
    :returns: list of strings in issue file, or None if issue file cannot be read/split
    """
    if os.path.exists(filename):
        with codecs.open(filename, 'r', encoding=locale.getpreferredencoding()) as f:
            return f.read().split()
    return None


def read_os_release(filename="/etc/os-release"):
    """
    :returns: Dictonary of key value pairs from /etc/os-release, with quotes stripped from values
    """
    release_info = {}
    if os.path.exists(filename):
        with codecs.open(filename, 'r', encoding=locale.getpreferredencoding()) as f:
            for line in f:
                key, val = line.rstrip('\n').partition('=')[::2]
                release_info[key] = val.strip('"')
            return release_info
    return None


class OsNotDetected(Exception):
    """
    Exception to indicate failure to detect operating system.
    """
    pass


class OsDetector(object):
    """
    Generic API for detecting a specific OS.
    """
    def is_os(self):
        """
        :returns: if the specific OS which this class is designed to
          detect is present.  Only one version of this class should
          return for any version.
        """
        raise NotImplementedError("is_os unimplemented")

    def get_version(self):
        """
        :returns: standardized version for this OS. (ala Ubuntu Hardy Heron = "8.04")
        :raises: :exc:`OsNotDetected` if called on incorrect OS.
        """
        raise NotImplementedError("get_version unimplemented")

    def get_codename(self):
        """
        :returns: codename for this OS. (ala Ubuntu Hardy Heron = "hardy").  If codenames are not available for this OS, return empty string.
        :raises: :exc:`OsNotDetected` if called on incorrect OS.
        """
        raise NotImplementedError("get_codename unimplemented")


class LsbDetect(OsDetector):
    """
    Generic detector for Debian, Ubuntu, and Mint
    """
    def __init__(self, lsb_name, get_version_fn=None):
        self.lsb_name = lsb_name
        if hasattr(platform, "linux_distribution"):
            self.lsb_info = platform.linux_distribution(full_distribution_name=0)
        elif hasattr(platform, "dist"):
            self.lsb_info = platform.dist()
        else:
            self.lsb_info = None

    def is_os(self):
        return self.lsb_info is not None and self.lsb_info[0] == self.lsb_name

    def get_version(self):
        if self.is_os():
            return self.lsb_info[1]
        raise OsNotDetected('called in incorrect OS')

    def get_codename(self):
        if self.is_os():
            return self.lsb_info[2]
        raise OsNotDetected('called in incorrect OS')


class Debian(LsbDetect):

    def __init__(self, get_version_fn=None):
        super(Debian, self).__init__('debian', get_version_fn)

    def get_codename(self):
        if self.is_os():
            v = self.get_version()
            if v.startswith('7.'):
                return 'wheezy'
            if v.startswith('8.'):
                return 'jessie'
            if v.startswith('9.'):
                return 'stretch'
            if v.startswith('10.'):
                return 'buster'
            return ''


class FdoDetect(OsDetector):
    """
    Generic detector for operating systems implementing /etc/os-release, as defined by the os-release spec hosted at Freedesktop.org (Fdo):
    http://www.freedesktop.org/software/systemd/man/os-release.html
    Requires that the "ID", and "VERSION_ID" keys are set in the os-release file.

    Codename is parsed from the VERSION key if available: either using the format "foo, CODENAME" or "foo (CODENAME)."
    If the VERSION key is not present, the VERSION_ID is value is used as the codename.
    """
    def __init__(self, fdo_id):
        release_info = read_os_release()
        if release_info is not None and "ID" in release_info and release_info["ID"] == fdo_id:
            self.release_info = release_info
        else:
            self.release_info = None

    def is_os(self):
        return self.release_info is not None and "VERSION_ID" in self.release_info

    def get_version(self):
        if self.is_os():
            return self.release_info["VERSION_ID"]
        raise OsNotDetected("called in incorrect OS")

    def get_codename(self):
        if self.is_os():
            if "VERSION" in self.release_info:
                version = self.release_info["VERSION"]
                # FDO style: works with Fedora, Debian, Suse.
                if version.find("(") is not -1:
                    codename = version[version.find("(") + 1:version.find(")")]
                # Ubuntu style
                elif version.find(",") is not -1:
                    codename = version[version.find(",") + 1:].lstrip(' ').split()[0]
                # Indeterminate style
                else:
                    codename = version
                return codename.lower()
            else:
                return self.get_version()
        raise OsNotDetected("called in incorrect OS")


class OpenSuse(OsDetector):
    """
    Detect OpenSuse OS.
    """
    def __init__(self, brand_file="/etc/SuSE-brand", release_file="/etc/SuSE-release"):
        self._brand_file = brand_file
        self._release_file = release_file

    def is_os(self):
        os_list = read_issue(self._brand_file)
        return os_list and os_list[0] == "openSUSE"

    def get_version(self):
        if self.is_os() and os.path.exists(self._brand_file):
            with open(self._brand_file, 'r') as fh:
                os_list = fh.read().strip().split('\n')
                if len(os_list) == 2:
                    os_list = os_list[1].split(' = ')
                    if os_list[0] == "VERSION":
                        return os_list[1]
        raise OsNotDetected('cannot get version on this OS')

    def get_codename(self):
        if self.is_os() and os.path.exists(self._release_file):
            with open(self._release_file, 'r') as fh:
                os_list = fh.read().strip().split('\n')
                for line in os_list:
                    kv = line.split(' = ')
                    if kv[0] == "CODENAME":
                        return kv[1]
        raise OsNotDetected('called in incorrect OS')


class Fedora(OsDetector):
    """
    Detect Fedora OS.
    """
    def __init__(self, release_file="/etc/redhat-release", issue_file="/etc/issue"):
        self._release_file = release_file
        self._issue_file = issue_file

    def is_os(self):
        os_list = read_issue(self._release_file)
        return os_list and os_list[0] == "Fedora"

    def get_version(self):
        if self.is_os():
            os_list = read_issue(self._issue_file)
            idx = os_list.index('release')
            if idx > 0:
                return os_list[idx + 1]
        raise OsNotDetected('cannot get version on this OS')

    def get_codename(self):
        if self.is_os():
            os_list = read_issue(self._release_file)
            idx = os_list.index('release')
            matches = [x for x in os_list if x[0] == '(']
            codename = matches[0][1:]
            if codename[-1] == ')':
                codename = codename[:-1]
            return codename.lower()
        raise OsNotDetected('called in incorrect OS')


class Rhel(Fedora):
    """
    Detect Redhat OS.
    """
    def __init__(self, release_file="/etc/redhat-release"):
        self._release_file = release_file

    def is_os(self):
        os_list = read_issue(self._release_file)
        return os_list and os_list[:3] == ['Red', 'Hat', 'Enterprise']

    def get_version(self):
        if self.is_os():
            os_list = read_issue(self._release_file)
            idx = os_list.index('release')
            return os_list[idx + 1]
        raise OsNotDetected('called in incorrect OS')

    def get_codename(self):
        # taroon, nahant, tikanga, santiago, pensacola
        if self.is_os():
            os_list = read_issue(self._release_file)
            idx = os_list.index('release')
            matches = [x for x in os_list if x[0] == '(']
            codename = matches[0][1:]
            if codename[-1] == ')':
                codename = codename[:-1]
            return codename.lower()
        raise OsNotDetected('called in incorrect OS')


# Source: http://en.wikipedia.org/wiki/Mac_OS_X#Versions
_osx_codename_map = {
    4: 'tiger',
    5: 'leopard',
    6: 'snow',
    7: 'lion',
    8: 'mountain lion',
    9: 'mavericks',
    10: 'yosemite',
    11: 'el capitan',
    12: 'sierra',
    13: 'high sierra',
    14: 'mojave',
}


def _osx_codename(major, minor):
    if major != 10 or minor not in _osx_codename_map:
        raise OsNotDetected("unrecognized version: %s.%s" % (major, minor))
    return _osx_codename_map[minor]


class OSX(OsDetector):
    """
    Detect OS X
    """
    def __init__(self, sw_vers_file="/usr/bin/sw_vers"):
        self._sw_vers_file = sw_vers_file

    def is_os(self):
        return os.path.exists(self._sw_vers_file)

    def get_codename(self):
        if self.is_os():
            version = self.get_version()
            import distutils.version  # To parse version numbers
            try:
                ver = distutils.version.StrictVersion(version).version
            except ValueError:
                raise OsNotDetected("invalid version string: %s" % (version))
            return _osx_codename(*ver[0:2])
        raise OsNotDetected('called in incorrect OS')

    def get_version(self):
        if self.is_os():
            return _read_stdout([self._sw_vers_file, '-productVersion'])
        raise OsNotDetected('called in incorrect OS')


class QNX(OsDetector):
    '''
    Detect QNX realtime OS.
    @author: Isaac Saito
    '''
    def __init__(self, uname_file='/bin/uname'):
        '''
        @param uname_file: An executable that can be used for detecting
                           OS name and version.
        '''
        self._os_name_qnx = 'QNX'
        self._uname_file = uname_file

    def is_os(self):
        if os.path.exists(self._uname_file):
            std_out = _read_stdout([self._uname_file])
            return std_out.strip() == self._os_name_qnx
        else:
            return False

    def get_codename(self):
        if self.is_os():
            return ''
        raise OsNotDetected('called in incorrect OS')

    def get_version(self):
        if self.is_os() and os.path.exists(self._uname_file):
            return _read_stdout([self._uname_file, "-r"])
        raise OsNotDetected('called in incorrect OS')


class Arch(OsDetector):
    """
    Detect Arch Linux.
    """
    def __init__(self, release_file='/etc/arch-release'):
        self._release_file = release_file

    def is_os(self):
        return os.path.exists(self._release_file)

    def get_version(self):
        if self.is_os():
            return ""
        raise OsNotDetected('called in incorrect OS')

    def get_codename(self):
        if self.is_os():
            return ""
        raise OsNotDetected('called in incorrect OS')


class Manjaro(Arch):
    """
    Detect Manjaro.
    """
    def __init__(self, release_file='/etc/manjaro-release'):
        super(Manjaro, self).__init__(release_file)


class Centos(OsDetector):
    """
    Detect CentOS.
    """
    def __init__(self, release_file='/etc/redhat-release'):
        self._release_file = release_file

    def is_os(self):
        os_list = read_issue(self._release_file)
        return os_list and os_list[0] == 'CentOS'

    def get_version(self):
        if self.is_os():
            os_list = read_issue(self._release_file)
            idx = os_list.index('release')
            return os_list[idx + 1]
        raise OsNotDetected('called in incorrect OS')

    def get_codename(self):
        if self.is_os():
            os_list = read_issue(self._release_file)
            idx = os_list.index('release')
            matches = [x for x in os_list if x[0] == '(']
            codename = matches[0][1:]
            if codename[-1] == ')':
                codename = codename[:-1]
            return codename.lower()
        raise OsNotDetected('called in incorrect OS')


class Cygwin(OsDetector):
    """
    Detect Cygwin presence on Windows OS.
    """
    def is_os(self):
        return os.path.exists("/usr/bin/cygwin1.dll")

    def get_version(self):
        if self.is_os():
            return _read_stdout(['uname', '-r'])
        raise OsNotDetected('called in incorrect OS')

    def get_codename(self):
        if self.is_os():
            return ''
        raise OsNotDetected('called in incorrect OS')


class Gentoo(OsDetector):
    """
    Detect Gentoo OS.
    """
    def __init__(self, release_file="/etc/gentoo-release"):
        self._release_file = release_file

    def is_os(self):
        os_list = read_issue(self._release_file)
        return os_list and os_list[0] == "Gentoo" and os_list[1] == "Base"

    def get_version(self):
        if self.is_os():
            os_list = read_issue(self._release_file)
            return os_list[4]
        raise OsNotDetected('called in incorrect OS')

    def get_codename(self):
        if self.is_os():
            return ''
        raise OsNotDetected('called in incorrect OS')


class Funtoo(Gentoo):
    """
    Detect Funtoo OS, a Gentoo Variant.
    """
    def __init__(self, release_file="/etc/gentoo-release"):
        Gentoo.__init__(self, release_file)

    def is_os(self):
        os_list = read_issue(self._release_file)
        return os_list and os_list[0] == "Funtoo" and os_list[1] == "Linux"


class FreeBSD(OsDetector):
    """
    Detect FreeBSD OS.
    """
    def __init__(self, uname_file="/usr/bin/uname"):
        self._uname_file = uname_file

    def is_os(self):
        if os.path.exists(self._uname_file):
            std_out = _read_stdout([self._uname_file])
            return std_out.strip() == "FreeBSD"
        else:
            return False

    def get_version(self):
        if self.is_os() and os.path.exists(self._uname_file):
            return _read_stdout([self._uname_file, "-r"])
        raise OsNotDetected('called in incorrect OS')

    def get_codename(self):
        if self.is_os():
            return ''
        raise OsNotDetected('called in incorrect OS')


class Slackware(OsDetector):
    """
    Detect SlackWare Linux.
    """
    def __init__(self, release_file='/etc/slackware-version'):
        self._release_file = release_file

    def is_os(self):
        return os.path.exists(self._release_file)

    def get_version(self):
        if self.is_os():
            os_list = read_issue(self._release_file)
            return os_list[1]
        raise OsNotDetected('called in incorrect OS')

    def get_codename(self):
        if self.is_os():
            return ''
        raise OsNotDetected('called in incorrect OS')


class Windows(OsDetector):
    """
    Detect Windows OS.
    """
    def is_os(self):
        return platform.system() == "Windows"

    def get_version(self):
        if self.is_os():
            return platform.version()
        raise OsNotDetected('called in incorrect OS')

    def get_codename(self):
        if self.is_os():
            return platform.release()
        raise OsNotDetected('called in incorrect OS')


class OsDetect:
    """
    This class will iterate over registered classes to lookup the
    active OS and version
    """

    default_os_list = []

    def __init__(self, os_list=None):
        if os_list is None:
            os_list = OsDetect.default_os_list
        self._os_list = os_list
        self._os_name = None
        self._os_version = None
        self._os_codename = None
        self._os_detector = None
        self._override = False

    @staticmethod
    def register_default(os_name, os_detector):
        """
        Register detector to be used with all future instances of
        :class:`OsDetect`.  The new detector will have precedence over
        any previously registered detectors associated with *os_name*.

        :param os_name: OS key associated with OS detector
        :param os_detector: :class:`OsDetector` instance
        """
        OsDetect.default_os_list.insert(0, (os_name, os_detector))

    def detect_os(self, env=None):
        """
        Detect operating system.  Return value can be overridden by
        the :env:`ROS_OS_OVERRIDE` environment variable.

        :param env: override ``os.environ``
        :returns: (os_name, os_version, os_codename), ``(str, str, str)``
        :raises: :exc:`OsNotDetected` if OS could not be detected
        """
        if env is None:
            env = os.environ
        if 'ROS_OS_OVERRIDE' in env:
            splits = env["ROS_OS_OVERRIDE"].split(':')
            self._os_name = splits[0]
            if len(splits) > 1:
                self._os_version = splits[1]
                if len(splits) > 2:
                    self._os_codename = splits[2]
                else:
                    self._os_codename = ''
            else:
                self._os_version = self._os_codename = ''
            self._override = True
        else:
            for os_name, os_detector in self._os_list:
                if os_detector.is_os():
                    self._os_name = os_name
                    self._os_version = os_detector.get_version()
                    self._os_codename = os_detector.get_codename()
                    self._os_detector = os_detector
                    break

        if self._os_name:
            return self._os_name, self._os_version, self._os_codename
        else:  # No solution found
            attempted = [x[0] for x in self._os_list]
            raise OsNotDetected("Could not detect OS, tried %s" % attempted)

    def get_detector(self, name=None):
        """
        Get detector used for specified OS name, or the detector for this OS if name is ``None``.

        :raises: :exc:`KeyError`
        """
        if name is None:
            if not self._os_detector:
                self.detect_os()
            return self._os_detector
        else:
            try:
                return [d for d_name, d in self._os_list if d_name == name][0]
            except IndexError:
                raise KeyError(name)

    def add_detector(self, name, detector):
        """
        Add detector to list of detectors used by this instance.  *detector* will override any previous
        detectors associated with *name*.

        :param name: OS name that detector matches
        :param detector: :class:`OsDetector` instance
        """
        self._os_list.insert(0, (name, detector))

    def get_name(self):
        if not self._os_name:
            self.detect_os()
        return self._os_name

    def get_version(self):
        if not self._os_version:
            self.detect_os()
        return self._os_version

    def get_codename(self):
        if not self._os_codename:
            self.detect_os()
        return self._os_codename


OS_ALPINE = 'alpine'
OS_ARCH = 'arch'
OS_MANJARO = 'manjaro'
OS_CENTOS = 'centos'
OS_CYGWIN = 'cygwin'
OS_DEBIAN = 'debian'
OS_ELEMENTARY = 'elementary'
OS_ELEMENTARY_OLD = 'elementary'
OS_FEDORA = 'fedora'
OS_FREEBSD = 'freebsd'
OS_FUNTOO = 'funtoo'
OS_GENTOO = 'gentoo'
OS_LINARO = 'linaro'
OS_MINT = 'mint'
OS_NEON = 'neon'
OS_OPENSUSE = 'opensuse'
OS_TIZEN = 'tizen'
OS_OPENSUSE13 = 'opensuse'
OS_OSX = 'osx'
OS_QNX = 'qnx'
OS_RHEL = 'rhel'
OS_SLACKWARE = 'slackware'
OS_UBUNTU = 'ubuntu'
OS_WINDOWS = 'windows'

OsDetect.register_default(OS_ALPINE, FdoDetect("alpine"))
OsDetect.register_default(OS_ARCH, Arch())
OsDetect.register_default(OS_MANJARO, Manjaro())
OsDetect.register_default(OS_CENTOS, Centos())
OsDetect.register_default(OS_CYGWIN, Cygwin())
OsDetect.register_default(OS_DEBIAN, Debian())
OsDetect.register_default(OS_ELEMENTARY, LsbDetect('"elementary"'))
OsDetect.register_default(OS_ELEMENTARY_OLD, LsbDetect('"elementary OS"'))
OsDetect.register_default(OS_FEDORA, FdoDetect("fedora"))
OsDetect.register_default(OS_FREEBSD, FreeBSD())
OsDetect.register_default(OS_FUNTOO, Funtoo())
OsDetect.register_default(OS_GENTOO, Gentoo())
OsDetect.register_default(OS_LINARO, LsbDetect("Linaro"))
OsDetect.register_default(OS_MINT, LsbDetect("LinuxMint"))
OsDetect.register_default(OS_NEON, LsbDetect("neon"))
OsDetect.register_default(OS_OPENSUSE, OpenSuse())
OsDetect.register_default(OS_OPENSUSE13, OpenSuse(brand_file='/etc/SUSE-brand'))
OsDetect.register_default(OS_OPENSUSE, FdoDetect("opensuse"))
OsDetect.register_default(OS_TIZEN, FdoDetect("tizen"))
OsDetect.register_default(OS_OSX, OSX())
OsDetect.register_default(OS_QNX, QNX())
OsDetect.register_default(OS_RHEL, Rhel())
OsDetect.register_default(OS_SLACKWARE, Slackware())
OsDetect.register_default(OS_UBUNTU, LsbDetect("Ubuntu"))
OsDetect.register_default(OS_WINDOWS, Windows())


if __name__ == '__main__':
    detect = OsDetect()
    print("OS Name:     %s" % detect.get_name())
    print("OS Version:  %s" % detect.get_version())
    print("OS Codename: %s" % detect.get_codename())
