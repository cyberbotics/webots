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

"""This module contains the Settings class."""

import configparser
import os
import sys


class Settings(object):
    """Settings class."""

    settings = None

    @staticmethod
    def get_section(primarySection, secondarySection=None):
        """Return the appropriate section in the settings."""
        if Settings.settings is None:
            sys.stderr.write("Warning: Settings.get_section() called before Settings.init()\n")
            return None
        settingsSection = primarySection + '_' + secondarySection
        if secondarySection is None or settingsSection not in Settings.settings.sections():
            settingsSection = primarySection
            if settingsSection not in Settings.settings.sections():
                return None
        if (Settings.settings.has_option(settingsSection, 'ignore') and
                Settings.settings.get(settingsSection, 'ignore') == 'TRUE'):
            return None
        return settingsSection

    @staticmethod
    def init(file):
        """Initialize the settings."""
        if not os.path.exists(file):
            sys.exit("Error: configuration file '%s' does not exist.\n" % file)
        Settings.settings = configparser.ConfigParser()
        Settings.settings.read(file)

    @staticmethod
    def has_option(section, option):
        """Determine if a section has this option."""
        if Settings.settings is None:
            sys.stderr.write("Warning: Settings.has_option() called before Settings.init()\n")
            return False
        elif not Settings.settings.has_section(section):
            return False
        else:
            return Settings.settings.has_option(section, option)

    @staticmethod
    def getfloat(section, option):
        """Return a float corresponding to the option value."""
        if Settings.settings is None:
            sys.stderr.write("Warning: Settings.getfloat() called before Settings.init()\n")
            return None
        else:
            return Settings.settings.getfloat(section, option)

    @staticmethod
    def get(section, option):
        """Return the value corresponding to the option."""
        if Settings.settings is None:
            sys.stderr.write("Warning: Settings.get() called before Settings.init()\n")
            return None
        else:
            return Settings.settings.get(section, option)

    @staticmethod
    def getint(section, option):
        """Return an int corresponding to the option value."""
        if Settings.settings is None:
            sys.stderr.write("Warning: Settings.getint() called before Settings.init()\n")
            return None
        else:
            return Settings.settings.getint(section, option)

    @staticmethod
    def sections():
        """Return the list of sections in the settings."""
        if Settings.settings is None:
            sys.stderr.write("Warning: Settings.sections() called before Settings.init()\n")
            return None
        else:
            return Settings.settings.sections()
