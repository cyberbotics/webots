# Copyright 1996-2023 Cyberbotics Ltd.
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

"""Sets the environment so that the cache tests can be performed in any condition"""

import atexit
import http.server
import os
import sys
import threading
from pathlib import Path

if 'WEBOTS_HOME' in os.environ:
    WEBOTS_HOME = os.environ['WEBOTS_HOME'].replace('\\', '/')
else:
    raise RuntimeError('WEBOTS_HOME environmental variable is not set.')

# necessary in order to be able to run the cache-related tests in the test suite
if 'TESTS_HOME' in os.environ:
    ROOT_FOLDER = os.environ['TESTS_HOME']
else:
    ROOT_FOLDER = WEBOTS_HOME

# Start an HTTP server to serve the contents of ROOT_FOLDER


class WebotsHomeHTTPRequestHandler(http.server.SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs, directory=Path(ROOT_FOLDER))

    def log_request(self, code='-', size='-'):
        if (code == 200):
            return
        super().log_request(code=code, size=size)


HTTP_PORT = 8000
httpd = http.server.HTTPServer(('', HTTP_PORT), WebotsHomeHTTPRequestHandler)
atexit.register(http.server.HTTPServer.server_close, httpd)
httpdThread = threading.Thread(target=http.server.HTTPServer.serve_forever, args=[httpd], daemon=True)
httpdThread.start()


def update_cache_urls(revert=False):
    paths = []
    paths.extend((Path(ROOT_FOLDER) / 'tests' / 'cache').rglob('*.proto'))
    paths.extend((Path(ROOT_FOLDER) / 'tests' / 'cache').rglob('*.wbt'))

    for path in paths:
        with open(path, 'r') as fd:
            content = fd.read()

        if revert:
            content = content.replace(ROOT_FOLDER + '/', 'absolute://')
            content = content.replace(f'http://localhost:{HTTP_PORT}/', 'web://')
        else:
            content = content.replace('absolute://', ROOT_FOLDER + '/')
            content = content.replace('web://', f'http://localhost:{HTTP_PORT}/')

        with open(path, 'w', newline='\n') as fd:
            fd.write(content)


if __name__ == '__main__':
    if len(sys.argv) != 2:
        print('Action not provided, options: "setup", "reset"')
    else:
        if sys.argv[1] == "setup":
            update_cache_urls()
        if sys.argv[1] == "reset":
            update_cache_urls(True)
