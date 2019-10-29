#!/usr/bin/env python
'''Cleanup all Webots preferences.'''

import platform

osName = platform.system()
if (osName == 'Linux'):
    #  for n in `find $HOME/.config/Cyberbotics -name Webots-*.conf`; do
    #    echo Remove "$n";
    #    rm $n;
    #  done
    pass
elif (osName == 'Darwin'):
    #  for n in `find $HOME/Library/Preferences -name com.cyberbotics.* | sed 's:.*/::;s:.plist::'`; do
    #    echo Remove "$n";
    #    defaults remove $n;
    #    rm $HOME/Library/Preferences/$n.plist;
    #  done
    pass
else:
    sys.exit('Unsupported OS: ' + osName)
