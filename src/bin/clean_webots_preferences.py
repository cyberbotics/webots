#!/usr/bin/env python3
'''Cleanup all Webots preferences.'''

import sys
from pathlib import Path

assert sys.version_info >= (3, 5), 'At least Python 3.5 is required to run this script.'
home = str(Path.home())
assert home, 'Cannot find the user home directory.'
print(home)


def cleanupLinuxPreferences():
    print('Cleanup Linux preferences')
    #  for n in `find $HOME/.config/Cyberbotics -name Webots-*.conf`; do
    #    echo Remove "$n";
    #    rm $n;
    #  done
    pass


def cleanupMacOSPreferences():
    print('Cleanup macOS preferences')
    #  for n in `find $HOME/Library/Preferences -name com.cyberbotics.* | sed 's:.*/::;s:.plist::'`; do
    #    echo Remove "$n";
    #    defaults remove $n;
    #    rm $HOME/Library/Preferences/$n.plist;
    #  done
    pass


def cleanupWindowsPreferences():
    print('Cleanup Windows preferences')
    # TODO: Something with regex
    pass


if __name__ == "__main__":
    import platform
    osName = platform.system()
    if osName == 'Linux':
        cleanupLinuxPreferences()
    elif osName == 'Darwin':
        cleanupMacOSPreferences()
    elif osName == 'Windows':
        cleanupWindowsPreferences()
    else:
        sys.exit('Unsupported OS: ' + osName)
