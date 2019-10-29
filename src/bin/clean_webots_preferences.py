#!/usr/bin/env python3
'''Cleanup all the Webots preferences.'''

import platform
import subprocess
import sys
from pathlib import Path

assert sys.version_info >= (3, 5), 'At least Python 3.5 is required to run this script.'


def cleanupLinuxPreferences():
    preferencesDir = Path.home() / '.config' / 'Cyberbotics'
    for preferencesPath in preferencesDir.glob('Webots-*.conf'):
        print('Clear the "%s" preference...' % preferencesPath)
        preferencesPath = Path(preferencesPath)
        preferencesPath.unlink()


def cleanupMacOSPreferences():
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
