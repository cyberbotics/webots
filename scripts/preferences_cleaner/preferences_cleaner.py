#!/usr/bin/env python3

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

'''Cleanup all the Webots preferences.'''

from __future__ import print_function  # To display a correct error message when running from Python 2.
import platform
import sys

# Due to the use of pathlib and winreg.
assert sys.version_info >= (3, 5), 'Python 3.5 or later is required to run this script.'


def cleanupLinuxPreferences():
    from pathlib import Path

    preferencesDir = Path.home() / '.config' / 'Cyberbotics'
    for preferencesPath in preferencesDir.glob('Webots-*.conf'):
        print('Clearing the "%s" preferences...' % preferencesPath)
        preferencesPath = Path(preferencesPath)
        preferencesPath.unlink()


def cleanupMacOSPreferences():
    import subprocess
    from pathlib import Path

    preferencesDir = Path.home() / 'Library' / 'Preferences'
    for preferencesPath in preferencesDir.glob('com.cyberbotics.*'):
        preferencesPath = Path(preferencesPath)
        preferenceReference = preferencesPath.stem
        print('Clearing the "%s" preferences...' % preferenceReference)
        subprocess.run(['defaults', 'remove', preferenceReference])
        if preferencesPath.exists():
            preferencesPath.unlink()


def cleanupWindowsPreferences():
    import winreg
    try:
        def deleteKeyRecursively(key):
            k = winreg.OpenKey(winreg.HKEY_CURRENT_USER, key, 0, winreg.KEY_ALL_ACCESS)
            subKeys = []
            try:
                i = 0
                while True:
                    subKeys.append(key + '\\' + winreg.EnumKey(k, i))
                    i += 1
            except WindowsError:
                pass  # Reached the end of the key enum.
            winreg.CloseKey(k)
            for subKey in subKeys:
                deleteKeyRecursively(subKey)
            winreg.DeleteKey(winreg.HKEY_CURRENT_USER, key)

        print("Clearing the 'Software\\Cyberbotics' registry key...")
        deleteKeyRecursively('Software\\Cyberbotics')
    except FileNotFoundError:
        print("Nothing to clean.")
    except PermissionError:
        print("You don't have the right to delete the Cyberbotics registry key.", file=sys.stderr)


if __name__ == "__main__":
    osName = platform.system()
    print('Cleaning-up Webots preferences on %s...' % osName)
    if osName == 'Linux':
        cleanupLinuxPreferences()
    elif osName == 'Darwin':
        cleanupMacOSPreferences()
    elif osName == 'Windows':
        cleanupWindowsPreferences()
    else:
        sys.exit('Unsupported OS: ' + osName)
    print('Done.')
