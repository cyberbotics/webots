# Software License Agreement (BSD License)
#
# Copyright (c) 2010, Willow Garage, Inc.
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

"""
Representation/model of rosdistro format.
"""

import os
import re
import string
try:
    from urllib.request import urlopen
except ImportError:
    from urllib2 import urlopen
import yaml

from .common import ResourceNotFound
from .environment import get_etc_ros_dir

TARBALL_URI_EVAL = 'http://svn.code.sf.net/p/ros-dry-releases/code/download/stacks/$STACK_NAME/$STACK_NAME-$STACK_VERSION/$STACK_NAME-$STACK_VERSION.tar.bz2'
TARBALL_VERSION_EVAL = '$STACK_NAME-$STACK_VERSION'


class InvalidDistro(Exception):
    """
    Distro file data does not match specification.
    """
    pass


def distro_uri(distro_name):
    """
    Get distro URI of main ROS distribution files.

    :param distro_name: name of distro, e.g. 'diamondback'
    :returns: the SVN/HTTP URL of the specified distro.  This function should only be used
      with the main distros.
    """
    return "http://svn.code.sf.net/p/ros-dry-releases/code/trunk/distros/%s.rosdistro" % (distro_name)

def expand_rule(rule, stack_name, stack_ver, release_name):
    s = rule.replace('$STACK_NAME', stack_name)
    if stack_ver:
        s = s.replace('$STACK_VERSION', stack_ver)
    s = s.replace('$RELEASE_NAME', release_name)
    return s


class DistroStack(object):
    """Stores information about a stack release"""

    def __init__(self, stack_name, stack_version, release_name, rules):
        """
        :param stack_name: Name of stack
        :param stack_version: Version number of stack.
        :param release_name: name of distribution release.  Necessary for rule expansion.
        :param rules: raw '_rules' data.  Will be converted into appropriate vcs config instance.
        """
        self.name = stack_name
        self.version = stack_version
        self.release_name = release_name
        self._rules = rules
        self.repo = rules.get('repo', None)
        self.vcs_config = load_vcs_config(self._rules, self._expand_rule)

    def _expand_rule(self, rule):
        """
        Perform variable substitution on stack rule.
        """
        return expand_rule(rule, self.name, self.version, self.release_name)

    def __eq__(self, other):
        try:
            return self.name == other.name and \
                self.version == other.version and \
                self.vcs_config == other.vcs_config
        except AttributeError:
            return False


class Variant(object):
    """
    A variant defines a specific set of stacks ("metapackage", in Debian
    parlance). For example, "base", "pr2". These variants can extend
    another variant.
    """

    def __init__(self, variant_name, extends, stack_names, stack_names_implicit):
        """
        :param variant_name: name of variant to load from distro file, ``str``
        :param stack_names_implicit: full list of stacks implicitly included in this variant, ``[str]``
        :param raw_data: raw rosdistro data for this variant
        """
        self.name = variant_name
        self.extends = extends
        self._stack_names = stack_names
        self._stack_names_implicit = stack_names_implicit

    def get_stack_names(self, implicit=True):
        if implicit:
            return self._stack_names_implicit
        else:
            return self._stack_names

    # stack_names includes implicit stack names. Use get_stack_names()
    # to get explicit only
    stack_names = property(get_stack_names)


class Distro(object):
    """
    Store information in a rosdistro file.
    """

    def __init__(self, stacks, variants, release_name, version, raw_data):
        """
        :param stacks: dictionary mapping stack names to :class:`DistroStack` instances
        :param variants: dictionary mapping variant names to :class:`Variant` instances
        :param release_name: name of release, e.g. 'diamondback'
        :param version: version number of release
        :param raw_data: raw dictionary representation of a distro
        """
        self._stacks = stacks
        self.variants = variants
        self.release_name = release_name
        self.version = version
        self.raw_data = raw_data

    def get_stacks(self, released=False):
        """
        :param released: only included released stacks
        :returns: dictionary of stack names to :class:`DistroStack` instances in
          this distro.
        """
        if released:
            return self._get_released_stacks()
        else:
            return self._stacks.copy()

    def _get_released_stacks(self):
        retval = {}
        for s, obj in self._stacks.items():
            if obj.version:
                retval[s] = obj
        return retval

    # gets map of all stacks
    stacks = property(get_stacks)
    # gets maps of released stacks
    released_stacks = property(_get_released_stacks)


def load_distro(source_uri):
    """
    :param source_uri: source URI of distro file, or path to distro
      file.  Filename has precedence in resolution.

    :raises: :exc:`InvalidDistro` If distro file is invalid
    :raises: :exc:`ResourceNotFound` If file at *source_uri* is not found
    """
    try:
        # parse rosdistro yaml
        if os.path.isfile(source_uri):
            # load rosdistro file
            with open(source_uri) as f:
                raw_data = yaml.load(f.read())
        else:
            try:
                request = urlopen(source_uri)
            except Exception as e:
                raise ResourceNotFound('%s (%s)' % (str(e), source_uri))
            try:
                raw_data = yaml.load(request)
            except ValueError:
                raise ResourceNotFound(source_uri)
        if not type(raw_data) == dict:
            raise InvalidDistro("Distro must be a dictionary: %s" % (source_uri))
    except yaml.YAMLError as e:
        raise InvalidDistro(str(e))

    try:
        version = _distro_version(raw_data.get('version', '0'))
        release_name = raw_data['release']
        stacks = _load_distro_stacks(raw_data, release_name)
        variants = _load_variants(raw_data.get('variants', {}), stacks)
        return Distro(stacks, variants, release_name, version, raw_data)
    except KeyError as e:
        raise InvalidDistro("distro is missing required '%s' key" % (str(e)))


def _load_variants(raw_data, stacks):
    if not raw_data:
        return {}
    all_variants_raw_data = {}
    for v in raw_data:
        if type(v) != dict or len(v.keys()) != 1:
            raise InvalidDistro("invalid variant spec: %s" % v)
        variant_name = list(v.keys())[0]
        all_variants_raw_data[variant_name] = v[variant_name]
    variants = {}
    for variant_name in all_variants_raw_data.keys():
        variants[variant_name] = _load_variant(variant_name, all_variants_raw_data)

        # Disabling validation to support variants which include wet packages.
        # validate
        # for stack_name in variants[variant_name].get_stack_names(implicit=False):
        #     if stack_name not in stacks:
        #         raise InvalidDistro("variant [%s] refers to non-existent stack [%s]"%(variant_name, stack_name))
    return variants


def _load_variant(variant_name, all_variants_raw_data):
    variant_raw_data = all_variants_raw_data[variant_name]
    stack_names_implicit = list(variant_raw_data.get('stacks', []))
    extends = variant_raw_data.get('extends', [])
    if isinstance(extends, str):
        extends = [extends]
    for e in extends:
        parent_variant = _load_variant(e, all_variants_raw_data)
        stack_names_implicit = parent_variant.get_stack_names(implicit=True) + stack_names_implicit
    return Variant(variant_name, extends, variant_raw_data.get('stacks', []), stack_names_implicit)


def _load_distro_stacks(distro_doc, release_name):
    """
    :param distro_doc: dictionary form of rosdistro file, `dict`
    :returns: dictionary of stack names to :class:`DistroStack` instances, `{str : DistroStack}`
    :raises: :exc:`InvalidDistro` if distro_doc format is invalid
    """

    # load stacks and expand out uri rules
    stacks = {}
    try:
        stack_props = distro_doc['stacks']
        stack_props = stack_props or {}
        stack_names = [x for x in stack_props.keys() if not x[0] == '_']
    except KeyError:
        raise InvalidDistro("distro is missing required 'stacks' key")
    for stack_name in stack_names:
        stack_version = stack_props[stack_name].get('version', None)
        rules = _get_rules(distro_doc, stack_name)
        if not rules:
            raise InvalidDistro("no VCS rules for stack [%s]" % (stack_name))
        stacks[stack_name] = DistroStack(stack_name, stack_version, release_name, rules)
    return stacks


def _distro_version(version_val):
    """
    Parse distro version value, converting SVN revision to version value if necessary
    """
    version_val = str(version_val)
    # check for no keyword sub
    if version_val == '$Revision$':
        return 0
    m = re.search('\$Revision:\s*([0-9]*)\s*\$', version_val)
    if m is not None:
        version_val = 'r' + m.group(1)

    # Check that is a valid version string
    valid = string.ascii_letters + string.digits + '.+~'
    if False in (c in valid for c in version_val):
        raise InvalidDistro("Version string %s not valid" % version_val)
    return version_val


def distro_to_rosinstall(distro, branch, variant_name=None, implicit=True, released_only=True, anonymous=True):
    """
    :param branch: branch to convert for
    :param variant_name: if not None, only include stacks in the specified variant.
    :param implicit: if variant_name is provided, include full (recursive) dependencies of variant, default True
    :param released_only: only included released stacks, default True.
    :param anonymous: create for anonymous access rules
    :returns: rosinstall data in Python list format, ``[dict]``

    :raises: :exc:`KeyError` If branch is invalid or if distro is mis-configured
    """
    variant = distro.variants.get(variant_name, None)
    if variant_name:
        stack_names = set(variant.get_stack_names(implicit=implicit))
    else:
        stack_names = distro.released_stacks.keys()
    rosinstall_data = []
    for s in stack_names:
        if released_only and s not in distro.released_stacks:
            continue
        rosinstall_data.extend(distro.stacks[s].vcs_config.to_rosinstall(s, branch, anonymous))
    return rosinstall_data

################################################################################


def _get_rules(distro_doc, stack_name):
    """
    Retrieve rules from distro_doc for specified stack.  This operates on
    the raw distro dictionary document.

    :param distro_doc: rosdistro document, ``dict``
    :param stack_name: name of stack to get rules for, ``str``
    """
    # top-level named section
    named_rules_d = distro_doc.get('_rules', {})

    # other rules to search
    rules_d = [distro_doc.get('stacks', {}),
               distro_doc.get('stacks', {}).get(stack_name, {})]
    rules_d = [d for d in rules_d if '_rules' in d]

    # last rules wins
    if not rules_d:
        return None
    rules_d = rules_d[-1]

    update_r = rules_d.get('_rules', {})
    if type(update_r) == str:
        try:
            update_r = named_rules_d[update_r]
        except KeyError:
            raise InvalidDistro("no _rules named [%s]" % (update_r))
    if not type(update_r) == dict:
        raise InvalidDistro("invalid rules: %s %s" % (update_r, type(update_r)))
    return update_r

################################################################################


class VcsConfig(object):
    """
    Base representation of a rosdistro VCS rules configuration.
    """

    def __init__(self, type_):
        self.type = type_
        self.tarball_url = self.tarball_version = None

    def to_rosinstall(self, local_name, branch, anonymous):
        uri, version_tag = self.get_branch(branch, anonymous)
        if branch == 'release-tar':
            type_ = 'tar'
        else:
            type_ = self.type
        if version_tag:
            return [{type_: {"uri": uri, 'local-name': local_name, 'version': version_tag}}]
        else:
            return [({type_: {"uri": uri, 'local-name': local_name}})]

    def load(self, rules, rule_eval):
        """
        Initialize fields of this class based on the raw rosdistro
        *rules* data after applying *rule_eval* function (e.g. to
        replace variables in rules).

        :param rules: raw rosdistro rules entry, ``dict``
        :param rule_eval: function to evaluate rule values, ``fn(str) -> str``
        """
        self.tarball_url = rule_eval(TARBALL_URI_EVAL)
        self.tarball_version = rule_eval(TARBALL_VERSION_EVAL)

    def get_branch(self, branch, anonymous):
        """
        :raises: :exc:`ValueError` If branch is invalid
        """
        if branch == 'release-tar':
            return self.tarball_url, self.tarball_version
        else:
            raise ValueError(branch)

    def __eq__(self, other):
        return self.type == other.type and \
            self.tarball_url == other.tarball_url


class DvcsConfig(VcsConfig):
    """
    Configuration information for a distributed VCS-style repository.

    Configuration fields:

     * ``repo_uri``: base URI of repo
     * ``dev_branch``: git branch the code is developed
     * ``distro_tag``: a tag of the latest released code for a specific ROS distribution
     * ``release_tag``: a tag of the code for a specific release
    """

    def __init__(self, type_):
        super(DvcsConfig, self).__init__(type_)
        self.repo_uri = self.anon_repo_uri = None
        self.dev_branch = self.distro_tag = self.release_tag = None

    def load(self, rules, rule_eval):
        super(DvcsConfig, self).load(rules, rule_eval)

        self.repo_uri = rule_eval(rules['uri'])
        if 'anon-uri' in rules:
            self.anon_repo_uri = rule_eval(rules['anon-uri'])
        else:
            self.anon_repo_uri = self.repo_uri
        self.dev_branch = rule_eval(rules['dev-branch'])
        self.distro_tag = rule_eval(rules['distro-tag'])
        self.release_tag = rule_eval(rules['release-tag'])

    def get_branch(self, branch, anonymous):
        """
        :raises: :exc:`KeyError` Invalid branch parameter
        """
        if branch == 'release-tar':
            return super(DvcsConfig, self).get_branch(branch, anonymous)
        elif branch == 'devel':
            version_tag = self.dev_branch
        elif branch == 'distro':
            version_tag = self.distro_tag
        elif branch == 'release':
            version_tag = self.release_tag
        else:
            raise ValueError("invalid branch spec [%s]" % (branch))
        # occurs, for example, with unreleased stacks.  Only devel is valid
        if version_tag is None:
            raise ValueError("branch [%s] is not available for this config" % (branch))
        if anonymous:
            return self.anon_repo_uri, version_tag
        else:
            return self.repo_uri, version_tag

    def __eq__(self, other):
        return super(DvcsConfig, self).__eq__(other) and \
            self.repo_uri == other.repo_uri and \
            self.anon_repo_uri == other.anon_repo_uri and \
            self.dev_branch == other.dev_branch and \
            self.release_tag == other.release_tag and \
            self.distro_tag == other.distro_tag


class GitConfig(DvcsConfig):
    """
    Configuration information about an GIT repository. See parent class :class:`DvcsConfig` for more API information.
    """

    def __init__(self):
        super(GitConfig, self).__init__('git')


class HgConfig(DvcsConfig):
    """
    Configuration information about a Mercurial repository. See parent class :class:`DvcsConfig` for more API information.
    """

    def __init__(self):
        super(HgConfig, self).__init__('hg')


class BzrConfig(DvcsConfig):
    """
    Configuration information about an BZR repository.  See parent class :class:`DvcsConfig` for more API information.
    """

    def __init__(self):
        super(BzrConfig, self).__init__('bzr')


class SvnConfig(VcsConfig):
    """
    Configuration information about an SVN repository.

    Configuration fields:

     * ``dev``: where the code is developed
     * ``distro_tag``: a tag of the code for a specific ROS distribution
     * ``release_tag``: a tag of the code for a specific release
    """

    def __init__(self):
        super(SvnConfig, self).__init__('svn')
        self.dev = None
        self.distro_tag = None
        self.release_tag = None

        # anonymously readable version of URLs above. Some repos have
        # separate URLs for read-only vs. writable versions of repo
        # and many tools need to be able to read repos without
        # providing credentials.
        self.anon_dev = None
        self.anon_distro_tag = None
        self.anon_release_tag = None

    def load(self, rules, rule_eval):
        super(SvnConfig, self).load(rules, rule_eval)
        for k in ['dev', 'distro-tag', 'release-tag']:
            if k not in rules:
                raise KeyError("svn rules missing required %s key: %s" % (k, rules))
        self.dev = rule_eval(rules['dev'])
        self.distro_tag = rule_eval(rules['distro-tag'])
        self.release_tag = rule_eval(rules['release-tag'])

        # specify urls that are safe to anonymously read
        # from. Users must supply a complete set.
        if 'anon-dev' in rules:
            self.anon_dev = rule_eval(rules['anon-dev'])
            self.anon_distro_tag = rule_eval(rules['anon-distro-tag'])
            self.anon_release_tag = rule_eval(rules['anon-release-tag'])
        else:
            # if no login credentials, assume that anonymous is
            # same as normal keys.
            self.anon_dev = self.dev
            self.anon_distro_tag = self.distro_tag
            self.anon_release_tag = self.release_tag

    def get_branch(self, branch, anonymous):
        """
        :raises: :exc:`ValueError` If branch is invalid
        """
        if branch == 'release-tar':
            return super(SvnConfig, self).get_branch(branch, anonymous)
        else:
            key_map = dict(devel='dev', distro='distro_tag', release='release_tag')
            if branch not in key_map:
                raise KeyError("invalid branch spec [%s]" % (branch))
            attr_name = key_map[branch]
            if anonymous:
                attr_name = 'anon_' + attr_name
            uri = getattr(self, attr_name)
        # occurs, for example, with unreleased stacks.  Only devel is valid
        if uri is None:
            raise ValueError("branch [%s] is not available for this config" % (branch))
        return uri, None

    def __eq__(self, other):
        return super(SvnConfig, self).__eq__(other) and \
            self.dev == other.dev and \
            self.distro_tag == other.distro_tag and \
            self.release_tag == other.release_tag and \
            self.anon_dev == other.anon_dev and \
            self.anon_distro_tag == other.anon_distro_tag and \
            self.anon_release_tag == other.anon_release_tag


_vcs_configs = {
    'svn': SvnConfig,
    'git': GitConfig,
    'hg': HgConfig,
    'bzr': BzrConfig,
}


def get_vcs_configs():
    """
    :returns: Dictionary of supported :class:`VcsConfig` instances.
      Key is the VCS type name, e.g. 'svn'. ``{str: VcsConfig}``
    """
    return _vcs_configs.copy()


def load_vcs_config(rules, rule_eval):
    """
    Factory for creating :class:`VcsConfig` subclass based on
    rosdistro _rules data.

    :param rules: rosdistro rules data
    :param rules_eval: Function to apply to rule values, e.g. to
      convert variables.  ``fn(str)->str``
    :returns: :class:`VcsConfig` subclass instance with interpreted rules data.
    """
    vcs_config = None
    for k, clazz in _vcs_configs.items():
        if k in rules:
            vcs_config = clazz()
            vcs_config.load(rules[k], rule_eval)
            break
    return vcs_config


def _current_distro_electric_parse_roscore(roscore_file):
    if not os.path.exists(roscore_file):
        return None
    import xml.dom.minidom
    try:
        dom = xml.dom.minidom.parse(roscore_file)
        tags = dom.getElementsByTagName("param")
        for t in tags:
            if t.hasAttribute('name') and t.getAttribute('name') == 'rosdistro':
                return t.getAttribute('value')
    except:
        return None


# for < fuerte, retrieve from roscore file
def _current_distro_electric(env=None):
    if env is None:
        env = os.environ
    from . import RosPack, get_ros_paths
    rospack = RosPack(get_ros_paths(env))
    # there's some chance that the location of this file changes in the future
    try:
        roscore_file = os.path.join(rospack.get_path('roslaunch'), 'roscore.xml')
        return _current_distro_electric_parse_roscore(roscore_file)
    except:
        return None


def current_distro_codename(env=None):
    """
    Get the currently active ROS distribution codename, e.g. 'fuerte'

    :param env: override os.environ, ``dict``
    """
    if env is None:
        env = os.environ

    # ROS_DISTRO is only used in ros catkin buildspace.  It is not
    # meant to be well publicized and thus is not declared in
    # rospkg.environment.
    if 'ROS_DISTRO' in env:
        return env['ROS_DISTRO']

    # check for /etc/ros/distro file
    distro_name = None
    etc_ros = get_etc_ros_dir(env=env)
    distro_file = os.path.join(etc_ros, 'distro')
    if os.path.isfile(distro_file):
        with open(distro_file, 'r') as f:
            distro_name = f.read().strip()

    # fallback logic for pre-Fuerte
    if distro_name is None:
        distro_name = _current_distro_electric(env=env)

    return distro_name
