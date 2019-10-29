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

'''Cleanup all the Webots preferences.'''

import platform
import sys

# Due to the use of pathlib.
assert sys.version_info >= (3, 5), 'At least Python 3.5 is required to run this script.'


def cleanupLinuxPreferences():
    from pathlib import Path

    preferencesDir = Path.home() / '.config' / 'Cyberbotics'
    for preferencesPath in preferencesDir.glob('Webots-*.conf'):
        print('Clear the "%s" preference...' % preferencesPath)
        preferencesPath = Path(preferencesPath)
        preferencesPath.unlink()


def cleanupMacOSPreferences():
    import subprocess
    from pathlib import Path

    preferencesDir = Path.home() / 'Library' / 'Preferences'
    for preferencesPath in preferencesDir.glob('com.cyberbotics.*'):
        preferencesPath = Path(preferencesPath)
        preferenceReference = preferencesPath.stem
        print('Clear the "%s" preference...' % preferenceReference)
        feedback = subprocess.run(['defaults', 'remove', preferenceReference])
        assert feedback.returncode == 0, 'Issue occured when removing the "%s" preference.'
        preferencesPath.unlink(missing_ok=True)


def cleanupWindowsPreferences():
    # TODO: Something with regex
    pass


if __name__ == "__main__":
    osName = platform.system()
    print('Cleanup %s preferences...' % osName)
    if osName == 'Linux':
        cleanupLinuxPreferences()
    elif osName == 'Darwin':
        cleanupMacOSPreferences()
    elif osName == 'Windows':
        cleanupWindowsPreferences()
    else:
        sys.exit('Unsupported OS: ' + osName)
    print('Done.')
