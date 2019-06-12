#!/usr/bin/env python

# Copyright 1996-2019 Cyberbotics Ltd.
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

"""Uploads the content of 'WEBOTS_HOME/distribution' to a Github release."""

import datetime
import os
import sys
from github import Github

if 'GITHUB_API_KEY' not in os.environ:
    sys.exit('GITHUB_API_KEY environment variable is not set.')

g = Github(os.environ['GITHUB_API_KEY'])
repo = g.get_repo("omichel/webots")
releaseExists = False
now = datetime.datetime.now()
title = 'Webots Nightly Build (%d-%d-%d)' % (now.day, now.month, now.year)
tag = 'nightly_%d_%d_%d' % (now.day, now.month, now.year)
if 'TRAVIS_TAG' in os.environ:
    print(os.environ['TRAVIS_TAG'])
#    tag = os.environ['TRAVIS_TAG']
#    title = tag
for release in repo.get_releases():
    print([release.title, release.prerelease, release.published_at])  # TODO: remove
    if release.title == title:
        releaseExists = True
        break

if not releaseExists:
    message = 'This is a nightly build of Webots from "%s" branch' % os.environ['TRAVIS_BRANCH']
    print('Creating release "%s" with tag "%s" on commit "%s"' % (title, tag, os.environ['TRAVIS_COMMIT']))
    repo.create_git_tag_and_release(tag=tag,
                                    tag_message=title,
                                    release_name=title,
                                    release_message=message,
                                    object=os.environ['TRAVIS_COMMIT'],
                                    type='commit',
                                    draft=True,  # TODO: prerelase instead
                                    prerelease=False)

for file in os.listdir(os.path.join(os.environ['WEBOTS_HOME'], 'distribution')):
    print(file)

for release in repo.get_releases():
    if release.title == title:
        for file in os.listdir(os.path.join(os.environ['WEBOTS_HOME'], 'distribution')):
            path = os.path.join(os.environ['WEBOTS_HOME'], 'distribution', file)
            if file != '.gitignore' and not os.path.isdir(path):
                print('Uploading "%s"' % file)
                release.upload_asset(path)
        break
print('Upload finished.')
