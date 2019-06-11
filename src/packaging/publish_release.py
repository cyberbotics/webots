#!/usr/bin/env python

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
for release in repo.get_releases():
    print([release.title, release.prerelease, release.published_at])  # TODO: remove
    if release.title == title:
        releaseExists = True
        break

if not releaseExists:
    message = 'This is a nightly build of Webots from "%s" branch' % os.environ['TRAVIS_BRANCH']
    repo.create_git_tag_and_release(tag=tag,
                                    tag_message=title,
                                    release_name=title,
                                    release_message=message,
                                    object=os.environ['TRAVIS_COMMIT'],
                                    type='commit',
                                    draft=True,  # TODO: prerelase instead
                                    prerelease=False)

for release in repo.get_releases():
    if release.title == title:
        for file in os.listdir(os.path.join(os.environ['WEBOTS_HOME'], 'distribution')):
            path = os.path.join(os.environ['WEBOTS_HOME'], 'distribution', file)
            if file != '.gitignore' and not os.path.isdir(path):
                release.upload_asset(path)
