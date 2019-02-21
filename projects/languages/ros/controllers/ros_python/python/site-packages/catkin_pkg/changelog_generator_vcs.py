# Software License Agreement (BSD License)
#
# Copyright (c) 2013, Open Source Robotics Foundation, Inc.
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
#  * Neither the name of Open Source Robotics Foundation, Inc. nor
#    the names of its contributors may be used to endorse or promote
#    products derived from this software without specific prior
#    written permission.
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

"""Extract log information from repositories."""

import os
import re
import shutil
import subprocess
import tempfile


class Tag(object):

    def __init__(self, name, timestamp=None):
        self.name = name
        self.timestamp = timestamp


class LogEntry(object):

    def __init__(self, msg, affected_paths, author):
        self.msg = msg
        self.author = author
        self._affected_paths = [p for p in affected_paths if p]

    def affects_path(self, path):
        for apath in self._affected_paths:
            # if the path is the root of the repository
            # it is affected by all changes
            if path == '.':
                return True
            if apath.startswith(os.path.join(path, '')):
                return True
        return False


class VcsClientBase(object):

    def __init__(self, path):
        self.path = path

    def get_tags(self):
        raise NotImplementedError()

    def get_latest_tag_name(self):
        raise NotImplementedError()

    def get_log_entries(self, from_tag, to_tag, skip_merges=False):
        raise NotImplementedError()

    def replace_repository_references(self, line):
        return line

    def _find_executable(self, file_name):
        for path in os.getenv('PATH').split(os.path.pathsep):
            file_path = os.path.join(path, file_name)
            if os.path.isfile(file_path):
                return file_path
        return None

    def _run_command(self, cmd, env=None):
        cwd = os.path.abspath(self.path)
        result = {'cmd': ' '.join(cmd), 'cwd': cwd}
        try:
            proc = subprocess.Popen(cmd, cwd=cwd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, env=env)
            output, _ = proc.communicate()
            result['output'] = output.rstrip().decode('utf-8')
            result['returncode'] = proc.returncode
        except subprocess.CalledProcessError as e:
            result['output'] = e.output
            result['returncode'] = e.returncode
        return result

    def _truncate_timestamps(self, tags):
        # truncate timestamps to shortest unique representation
        # - date only
        # - date including hours and minutes
        # - date include hours, minutes and seconds
        lengths = [10, 16, 19]
        for length in lengths:
            # filter tags which have not been truncated yet
            considered_tags = [t for t in tags if len(t.timestamp) > length]
            # count tags which timestamps have the same truncated representation
            grouped_by_timestamp = {}
            for t in considered_tags:
                truncated_timestamp = t.timestamp[:length]
                if truncated_timestamp not in grouped_by_timestamp:
                    grouped_by_timestamp[truncated_timestamp] = []
                grouped_by_timestamp[truncated_timestamp].append(t)
            # truncate timestamp of tags which are unique
            for truncated_timestamp, similar_tags in grouped_by_timestamp.items():
                if len(similar_tags) == 1:
                    similar_tags[0].timestamp = truncated_timestamp


class GitClient(VcsClientBase):

    type = 'git'  # noqa: A003

    def __init__(self, path):
        super(GitClient, self).__init__(path)
        self._executable = self._find_executable('git')
        self._repo_hosting = None
        self._github_base_url = 'https://github.com/'
        self._github_path = None

    # query author
    def _get_author(self, hash_):
        cmd = [self._executable, 'log', hash_, '-n', '1', '--format=format:%aN']
        result = self._run_command(cmd)
        if result['returncode']:
            raise RuntimeError('Could not fetch author:\n%s' % result['output'])
        return result['output']

    def get_tags(self):
        # Get a decorated log, use the refnames to find the ancestor tags
        cmd_tag = [self._executable, 'log', '--simplify-by-decoration', '--decorate', '--pretty=oneline']
        result_tag = self._run_command(cmd_tag)
        if result_tag['returncode']:
            raise RuntimeError('Could not fetch tags:\n%s' % result_tag['output'])
        # Parse a comma-separated list of refname decorators out of the log
        decorations = ', '.join(re.findall('^[a-f0-9]+ \(([^)]*)\) .', result_tag['output'], re.MULTILINE)) + ','
        # Extract only refnames that are tags
        tag_names = re.findall('tag: ([^,]+)[,]', decorations)

        tags = []
        for tag_name in tag_names:
            cmd = [self._executable, 'log', tag_name, '-n', '1', '--format=format:%ai']
            result = self._run_command(cmd)
            if result['returncode']:
                raise RuntimeError('Could not fetch timestamp:\n%s' % result['output'])
            tags.append(Tag(tag_name, result['output']))
        self._truncate_timestamps(tags)
        return tags

    def get_latest_tag_name(self):
        cmd_describe = [self._executable, 'describe', '--abbrev=0', '--tags']
        result_describe = self._run_command(cmd_describe)
        if result_describe['returncode']:
            raise RuntimeError('Could not fetch latest tag:\n%s' % result_describe['output'])
        tag_name = result_describe['output']
        return tag_name

    def get_log_entries(self, from_tag, to_tag, skip_merges=False):
        # query all hashes in the range
        cmd = [self._executable, 'log']
        if from_tag or to_tag:
            cmd.append('%s%s' % ('%s..' % to_tag if to_tag else '', from_tag if from_tag else ''))
        cmd.append('--format=format:%H')
        if skip_merges:
            cmd.append('--no-merges')
        result = self._run_command(cmd)
        if result['returncode']:
            raise RuntimeError('Could not fetch commit hashes:\n%s' % result['output'])

        log_entries = []
        if result['output']:
            # query further information for each changeset
            hashes = result['output'].splitlines()
            for hash_ in hashes:
                # query commit message
                cmd = [self._executable, 'log', hash_, '-n', '1', '--format=format:%B']
                result = self._run_command(cmd)
                if result['returncode']:
                    raise RuntimeError('Could not fetch commit message:\n%s' % result['output'])
                if result['output'] == from_tag:
                    continue
                msg = result['output']
                # query affected paths
                cmd = [self._executable, 'show', '--first-parent', hash_, '--name-only', '--format=format:""']
                result = self._run_command(cmd)
                if result['returncode']:
                    raise RuntimeError('Could not fetch affected paths:\n%s' % result['output'])
                affected_paths = result['output'].splitlines()
                log_entries.append(LogEntry(msg, affected_paths, self._get_author(hash_)))
        return log_entries

    def replace_repository_references(self, line):
        if self._repo_hosting is None:
            self._repo_hosting = False
            try:
                self._determine_repo_hosting()
            except RuntimeError:
                pass
        if self._repo_hosting == 'github':
            line = self._replace_github_issue_references(line)
        return line

    def _determine_repo_hosting(self):
        cmd = [self._executable, 'config', '--get', 'remote.origin.url']
        result = self._run_command(cmd)
        if result['returncode']:
            raise RuntimeError('Could not fetch remote url:\n%s' % result['output'])

        # detect github hosting
        prefixes = ['git@github.com:', 'https://github.com/', 'git://github.com/']
        for prefix in prefixes:
            if result['output'].startswith(prefix):
                self._repo_hosting = 'github'
                path = result['output'][len(prefix):]
                if path.endswith('.git'):
                    path = path[:-4]
                self._github_path = path
                break

    def _replace_github_issue_references(self, line):
        valid_name = '[\\w\._-]+'
        issue_pattern = '#(\\d+)'

        def replace_issue_number(match):
            issue_url = self._github_base_url
            if match.group(1):
                path = match.group(1)
                issue_url += path
            else:
                path = ''
                issue_url += self._github_path
            issue_number = match.group(2)
            issue_url += '/issues/' + issue_number
            return '`%s#%s <%s>`_' % (path, issue_number, issue_url)
        line = re.sub(('(%s/%s)?' % (valid_name, valid_name)) + issue_pattern, replace_issue_number, line)
        return line


class HgClient(VcsClientBase):

    type = 'hg'  # noqa: A003

    def __init__(self, path):
        super(HgClient, self).__init__(path)
        self._executable = self._find_executable('hg')

    # query author
    def _get_author(self, hash_):
        cmd = [self._executable, 'log', '-r', hash_, '--template', '{author}']
        result = self._run_command(cmd)
        if result['returncode']:
            raise RuntimeError('Could not fetch author:\n%s' % result['output'])
        return result['output']

    def get_tags(self):
        cmd_tag = [self._executable, 'tags', '-q']
        result_tag = self._run_command(cmd_tag)
        if result_tag['returncode']:
            raise RuntimeError('Could not fetch tags:\n%s' % result_tag['output'])
        tag_names = result_tag['output'].splitlines()

        tags = []
        for tag_name in tag_names:
            cmd = [self._executable, 'log', '-r', tag_name, '--template', '{date|isodatesec}']
            result = self._run_command(cmd)
            if result['returncode']:
                raise RuntimeError('Could not fetch timestamp:\n%s' % result['output'])
            tags.append(Tag(tag_name, result['output']))
        self._truncate_timestamps(tags)
        return tags

    def get_latest_tag_name(self):
        cmd_log = [self._executable, 'log', '--rev', '.', '--template', '{latesttag}']
        result_log = self._run_command(cmd_log)
        if result_log['returncode']:
            raise RuntimeError('Could not fetch latest tag:\n%s' % result_log['output'])
        tag_name = result_log['output']
        if tag_name == 'null':
            raise RuntimeError('Could not find latest tagn')
        return tag_name

    def get_log_entries(self, from_tag, to_tag, skip_merges=False):
        # query all hashes in the range
        # ascending chronological order since than it is easier to handle empty tag names
        revrange = '%s:%s' % ((to_tag if to_tag else ''), (from_tag if from_tag else 'tip'))
        if to_tag:
            revrange += '-%s' % to_tag
        if from_tag:
            revrange += '-%s' % from_tag
        cmd = [self._executable, 'log', '-r', revrange, '--template', '{rev}\n']
        result = self._run_command(cmd)
        if result['returncode']:
            raise RuntimeError('Could not fetch commit hashes:\n%s' % result['output'])

        tmp_base = tempfile.mkdtemp('-hg-style')
        try:
            style_file = os.path.join(tmp_base, 'hg-changeset-files-per-line.style')
            with open(style_file, 'w') as f:
                f.write("changeset = '{files}'\n")
                f.write("file = '{file}\\n'\n")

            log_entries = []
            if result['output']:
                # query further information for each changeset
                revs = reversed(result['output'].splitlines())
                for rev in revs:
                    # query commit message
                    cmd = [self._executable, 'log', '-r', rev, '-l', '1', '--template', '{desc}']
                    result = self._run_command(cmd)
                    if result['returncode']:
                        raise RuntimeError('Could not fetch commit message:\n%s' % result['output'])
                    if result['output'] == from_tag:
                        continue
                    msg = result['output']
                    # query affected paths
                    cmd = [self._executable, 'log', '-r', rev, '-l', '1', '--style', style_file]
                    result = self._run_command(cmd)
                    if result['returncode']:
                        raise RuntimeError('Could not fetch affected paths:\n%s' % result['output'])
                    affected_paths = result['output'].splitlines()
                    log_entries.append(LogEntry(msg, affected_paths, self._get_author(rev)))
        finally:
            shutil.rmtree(tmp_base)
        return log_entries


def get_vcs_client(base_path):
    vcs_clients = []
    vcs_clients.append(GitClient)
    vcs_clients.append(HgClient)
    client_types = [c.type for c in vcs_clients]
    if len(client_types) != len(set(client_types)):
        raise RuntimeError('Multiple vcs clients share the same type: %s' % ', '.join(sorted(client_types)))

    for vcs_client in vcs_clients:
        if os.path.exists(os.path.join(base_path, '.%s' % vcs_client.type)):
            return vcs_client(base_path)
    raise RuntimeError('Could not detect repository type - currently supports: %s' % ', '.join([c.type for c in vcs_clients]))
