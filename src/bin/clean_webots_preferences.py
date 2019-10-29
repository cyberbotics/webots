#!/usr/bin/env python
'''Cleanup all Webots preferences.'''


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
    import sys
    osName = platform.system()
    if osName == 'Linux':
        cleanupLinuxPreferences()
    elif osName == 'Darwin':
        cleanupMacOSPreferences()
    elif osName == 'Windows':
        cleanupWindowsPreferences()
    else:
        sys.exit('Unsupported OS: ' + osName)
