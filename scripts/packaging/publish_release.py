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

"""Uploads the content of 'WEBOTS_HOME/distribution' to a Github release."""

import datetime
import optparse
import os
import re
import requests
import urllib3
import sys
import time
from github import Github
from github import GithubException
from github.GithubException import UnknownObjectException

optParser = optparse.OptionParser(
    usage="usage: %prog --key=... --repo=cyberbotics/webots --branch=develop --commit=... [--tag=...]")
optParser.add_option("--key", dest="key", default="", help="specifies the repo access key")
optParser.add_option("--repo", dest="repo", default="cyberbotics/webots", help="specifies the repo")
optParser.add_option("--tag", dest="tag", default="", help="optionnally specifies a tag")
optParser.add_option("--branch", dest="branch", default="", help="specifies the branch from which is uploaded the release.")
optParser.add_option("--commit", dest="commit", default="", help="specifies the commit from which is uploaded the release.")
options, args = optParser.parse_args()

# status codes returned on random github server errors
status_forcelist = (500, 502, 504)
retry = urllib3.Retry(total=10, status_forcelist=status_forcelist)
g = Github(options.key, retry=retry)
repo = g.get_repo(options.repo)
releaseExists = False
now = datetime.datetime.now()
if now.hour <= 5:
    # Publish nightly build with previous day date even if it completes in the morning
    now = now - datetime.timedelta(hours=6)
if now.weekday() >= 5:
    print('Skipping nightly build for Saturday and Sunday.')
    sys.exit(0)

warningMessage = '\nIt might be unstable, for a stable version of Webots, please use the [latest official release]' \
                 '(https://github.com/cyberbotics/webots/releases/latest).'
if options.tag and not options.tag.startswith('refs/heads/'):
    tagName = options.tag.replace('refs/tags/', '')
    title = tagName
    message = 'This is a nightly build of Webots from the "%s" tag.%s' % (tagName, warningMessage)
    if tagName.startswith('nightly_'):
        print('Skipping nightly build tag.')
        sys.exit(0)
else:
    title = 'Webots Nightly Build (%d-%d-%d)' % (now.day, now.month, now.year)
    tagName = 'nightly_%d_%d_%d' % (now.day, now.month, now.year)
    branchName = options.branch.replace('refs/heads/', '')
    branchLink = '[%s](https://github.com/%s/blob/%s/docs/reference/changelog-r%d.md)' \
                 % (branchName, options.repo, options.commit, now.year)
    message = 'This is a nightly build of Webots from the following branch(es):\n  - %s\n%s' % (branchLink, warningMessage)

for release in repo.get_releases():
    match = re.match(r'Webots Nightly Build \((\d*)-(\d*)-(\d*)\)', release.title, re.MULTILINE)
    if release.title == title:
        releaseExists = True
        break
    elif match:
        date = now - datetime.datetime(year=int(match.group(3)), month=int(match.group(2)), day=int(match.group(1)))
        maxDay = 3
        if now.weekday() <= 1:  # Monday or tuesday
            maxDay += 2  # weekend day doesn't count
        if date > datetime.timedelta(days=maxDay, hours=12):  # keep only 3 nightly releases in total
            releaseTagName = release.tag_name
            print('Deleting release "%s"' % release.title)
            release.delete_release()
            try:
                ref = repo.get_git_ref('tags/' + releaseTagName)
                if ref:
                    print('Deleting tag "%s"' % releaseTagName)
                    ref.delete()
            except UnknownObjectException:
                pass  # tag was already deleted

if not releaseExists:
    print('Creating release "%s" with tag "%s" on commit "%s"' % (title, tagName, options.commit))
    draft = False if tagName.startswith('nightly_') else True
    tagExists = False
    for tag in repo.get_tags():
        if tag.name == tagName:
            tagExists = True
            break

    if tagExists:
        print('Tag "%s" already exists.' % (tagName))
        try:
            repo.create_git_release(tag=tagName,
                                    name=title,
                                    message=message,
                                    draft=draft,
                                    prerelease=True,
                                    target_commitish=options.commit)
        except GithubException as e:
            print('Creation of release failed: ', e.data)
    else:
        try:
            repo.create_git_tag_and_release(tag=tagName,
                                            tag_message=title,
                                            release_name=title,
                                            release_message=message,
                                            object=options.commit,
                                            type='commit',
                                            draft=draft,
                                            prerelease=True)
        except GithubException as e:
            print('Creation of tag and release failed: ', e.data)

time.sleep(60)  # allow some delay between creating the tag and requesting the existing ones
releaseFound = False
for release in repo.get_releases():
    if release.title == title:
        releaseFound = True
        assets = {}
        for asset in release.get_assets():
            assets[asset.name] = asset
        releaseCommentModified = False
        if 'WEBOTS_HOME' in os.environ:
            rootPath = os.path.normpath(os.environ['WEBOTS_HOME'])
        else:
            rootPath = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
        for file in os.listdir(os.path.join(rootPath, 'distribution')):
            path = os.path.join(rootPath, 'distribution', file)
            if file != '.gitignore' and not os.path.isdir(path):
                if file in assets:
                    print('Asset "%s" already present in release "%s".' % (file, title))
                else:
                    print('Uploading "%s"' % file)
                    retryCount = 0
                    while retryCount < 5:
                        if retryCount > 0:
                            time.sleep(retryCount * 60)  # wait some minutes before retry
                        try:
                            release.upload_asset(path)
                            break
                        except requests.exceptions.ConnectionError:
                            print('Release upload failed due to connection error (remaining trials: %d)' % (4 - retryCount))
                        except requests.exceptions.HTTPError:
                            print('Release upload failed due to server error (remaining trials: %d)' % (4 - retryCount))
                        except GithubException as e:
                            print('Release upload failed due to GitHub error (remaining trials: %d)' % (4 - retryCount))
                            print('Error message', e)
                        except Exception as e:
                            print('Release upload failed due to unexpected error (remaining trials: %d)' % (4 - retryCount))
                            print('Error message', e)
                        retryCount += 1
                    if (releaseExists and tagName.startswith('nightly_') and not releaseCommentModified and
                            branchName not in release.body):
                        print('Updating release description')
                        releaseCommentModified = True
                        branchLink = '[%s](https://github.com/%s/blob/%s/docs/reference/changelog-r%d.md)' \
                                     % (branchName, options.repo, options.commit, now.year)
                        message = release.body.replace('branch(es):', 'branch(es):\n  - %s' % branchLink)
                        release.update_release(release.title, message, release.draft, release.prerelease, release.tag_name,
                                               release.target_commitish)
        break

if not releaseFound:  # if it does not exist, it should have been created by the script itself
    print('Error, release "%s" should exist by now but does not.' % title)
else:
    print('Upload finished.')
