#!/usr/bin/env python

# Copyright 1996-2022 Cyberbotics Ltd.
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

"""Generate and package the asset cache"""
import os
from pathlib import Path
import sys
import shutil
import hashlib
from datetime import date

EXTENSIONS = [
    'jpg', 'png', 'hdr', 'stl', 'dae', 'obj', 'mp3', 'wav', 'fbx', 'proto'
]

SKIPPED_DIRECTORIES = [
    '/robot_windows/', '/thymio2_aseba/'
]

# ensure WEBOTS_HOME is set and tag was provided
if 'WEBOTS_HOME' in os.environ:
    WEBOTS_HOME = os.environ['WEBOTS_HOME']
else:
    raise RuntimeError('Error, WEBOTS_HOME variable is not set.')


def generate_asset_cache(tag, version_name):
    folder_name = f'assets-{version_name}'

    # retrieve all assets
    assets = []
    for extension in EXTENSIONS:
        assets.extend(Path(WEBOTS_HOME + '/projects').rglob(f'*.{extension}'))

    # create and fill asset folder
    if os.path.exists(folder_name):
        shutil.rmtree(folder_name)

    os.mkdir(folder_name)
    with open(f'{folder_name}/log.txt', 'w') as log_file:
        log_file.write(f'Cache generated on {date.today()} with tag {tag}\n\n')

        for asset in assets:
            if any(x in str(asset) for x in SKIPPED_DIRECTORIES):
                continue

            # generate hash of the remote url
            remote_url = str(asset).replace(WEBOTS_HOME, rf'https://raw.githubusercontent.com/cyberbotics/webots/{tag}')
            hash = hashlib.sha1(remote_url.encode('utf-8')).hexdigest()
            # copy to asset folder
            shutil.copyfile(asset, f'./{folder_name}/{hash}')
            # store the url-hash equivalence for debugging purposes
            log_file.write(f'{hash} {remote_url}\n')

    # generate zip file
    shutil.make_archive(folder_name, 'zip', folder_name)
    shutil.move(f'{folder_name}.zip', f'{WEBOTS_HOME}/distribution/{folder_name}.zip')
    shutil.rmtree(folder_name)


if __name__ == "__main__":
    if len(sys.argv) != 3:
        sys.exit('''Missing one or more arguments.
            It should have the form: generate_asset_cache [commit sha or tag] [webots version name]''')
    else:
        tag = sys.argv[1]
        version_name = sys.argv[2]
    generate_asset_cache(tag, version_name)
