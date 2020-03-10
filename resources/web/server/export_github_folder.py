#!/usr/bin/env python3

# Copyright 1996-2020 Cyberbotics Ltd.
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

"""Webots project checkout from github."""

import argparse
import os
import sys


def main():
    """Checkout a Webots project from a github repository."""
    # The URL is provided as the argument of this script
    # It takes the form https://github.com/my_username/my_project_name/tree/my_tag_or_branch[/my_path]
    # If no path is provided, the project is assumed to be located at the root level of the repository.
    # If no /tree/ component is provided, the /tree/master branch is assumed
    # subversion is used to export the requested github folder


parser = argparse.ArgumentParser(description='Export a github repository.')
parser.add_argument('-v', '--version', action='version', version='1.0')
parser.add_argument('--url', required=True, help='url of the github repository to export, for example: ' +
                    'https://github.com/user/repo/tree/master/folder')
parser.add_argument('--output', default='.', help='output folder, default to current folder')
parser.add_argument('--tag', action='store_true', help='the url refers to a tag and not a branch')
args = parser.parse_args()
if not args.url.startswith('https://github.com/'):
    sys.exit('The URL argument should start with "https://github.com/".')
parts = args.url[19:].split('/')
length = len(parts)
if length < 2:
    sys.exit('Missing repository in URL.')
if length == 2 or parts[2] != 'tree':
    parts.insert(2, 'tree')
    parts.insert(3, 'master')
    length += 2
elif length == 3:
    sys.exit('Missing tree value.')
username = parts[0]
repository = parts[1]
if args.tag:
    tag = parts[3]
    branch = ''
else:
    branch = parts[3]
    tag = ''
folder = '/'.join(parts[4:])
f = '/' + folder if folder else ''
url = 'https://github.com/' + username + '/' + repository + '/'
if branch == 'master':
    url += 'trunk'
elif branch == '':
    url += 'tags/' + tag
else:
    url += 'branches/' + branch
if folder:
    url += '/' + folder
command = 'svn export ' + url
path = os.getcwd()
os.chdir(args.output)
os.system(command)
if branch == 'master' and folder == '':
    os.rename('trunk', repository)
os.chdir(path)
