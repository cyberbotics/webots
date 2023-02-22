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

"""This module contains the SpeedLimit class."""
import re


class SpeedLimit:
    """Helper functions to compute the speed limit of a road."""

    def __init__(self, country):
        """Constructor: display a warning if the country is not supported."""
        self.country = country  # Expected: 'CH', 'US', etc.

        if country is not None and country not in ['FR', 'DE', 'JP', 'CH', 'GB', 'US']:
            print('Warning: The exported Road.speedLimit may be wrong because the maxspeed rules is not'
                  ' yet implemented for this country.')
            print('  Supported countries:')
            print('  - France')
            print('  - Germany(default)')
            print('  - Japan')
            print('  - Switzerland')
            print('  - United Kingdom')
            print('  - United States')
            print('Please contact support@cyberbotics.com to support your country (you may also safely ignore this warning).')

    def compute_speed_limit(self, road):
        """Compute the speed limit, based on the OSM tags of an OSM way."""
        if 'maxspeed' in road.tags:
            # Case 1: 'maxspeed' is specified in the OSM tag.
            speed = road.tags['maxspeed']
            for tag, value in SpeedLimit.speedTags.items():
                if isinstance(value, (int, float)) and speed == tag:
                    # The OSM 'maxspeed' is defined by a tag.
                    return value / 3.6
                elif isinstance(value, tuple):
                    if speed == tag:
                        if road.is_inside_a_city():
                            return value[0] / 3.6
                        else:
                            return value[1] / 3.6

            # Case 2: The OSM 'maxspeed' is defined as a specific speed, possibly containing the 'mph' suffix.
            m = re.match(r'(\d*)\s*(.*)?', speed)
            if m is not None:
                speedLimit = int(m.group(1)) / 3.6
                if m.group(2).lower() == 'mph':
                    speedLimit = int(m.group(1)) * 0.44704
                if speedLimit > 0.0:
                    return speedLimit

        # Case 3: 'maxspeed' is not specified, then the default speed should be computed.
        # A rule per country should be implemented.
        if self.country is not None and self.country == 'CH':
            # Switzerland.
            # ref: http://wiki.openstreetmap.org/wiki/OSM_tags_for_routing/Maxspeed#Switzerland
            if 'motorway' in road.type.lower():
                return 120.0 / 3.6
            elif 'trunk' in road.type.lower():
                return 100.0 / 3.6
            elif 'primary' in road.type.lower() or 'secondary' in road.type.lower() or 'tertiary' in road.type.lower():
                return 80.0 / 3.6
            elif 'residential' in road.type.lower():
                return 50.0 / 3.6
            elif 'living_street' in road.type.lower():
                return 30.0 / 3.6
            else:
                if road.is_inside_a_city():
                    return 50.0 / 3.6
                else:
                    return 80.0 / 3.6
        elif self.country is not None and self.country == 'FR':
            # France.
            # ref: http://wiki.openstreetmap.org/wiki/OSM_tags_for_routing/Maxspeed#France
            if 'motorway' in road.type.lower():
                return 130.0 / 3.6
            elif 'trunk' in road.type.lower():
                return 110.0 / 3.6
            elif 'residential' in road.type.lower():
                return 50.0 / 3.6
            elif 'living_street' in road.type.lower():
                return 20.0 / 3.6
            else:
                if road.is_inside_a_city():
                    return 50.0 / 3.6
                else:
                    return 90.0 / 3.6
        elif self.country is not None and self.country == 'US':
            # United State (New York).
            # ref: http://wiki.openstreetmap.org/wiki/OSM_tags_for_routing/Maxspeed#United_States_of_America
            if 'motorway' in road.type.lower() or 'trunk' in road.type.lower():
                return 65.0 * SpeedLimit.milesFactor / 3.6
            elif 'residential' in road.type.lower():
                return 30.0 * SpeedLimit.milesFactor / 3.6
            else:
                return 55.0 * SpeedLimit.milesFactor / 3.6
        elif self.country is not None and self.country == 'GB':
            # United Kingdom.
            # ref: http://wiki.openstreetmap.org/wiki/OSM_tags_for_routing/Maxspeed#United_Kingdom
            if 'motorway' in road.type.lower() or 'trunk' in road.type.lower():
                return 70.0 * SpeedLimit.milesFactor / 3.6
            elif 'residential' in road.type.lower():
                return 30.0 * SpeedLimit.milesFactor / 3.6
            else:
                return 60.0 * SpeedLimit.milesFactor / 3.6
        elif self.country is not None and self.country == 'JP':
            # Japan.
            # ref: http://wiki.openstreetmap.org/wiki/OSM_tags_for_routing/Maxspeed#Japan
            if 'motorway' in road.type.lower() or 'trunk' in road.type.lower():
                return 100.0 / 3.6
            else:
                return 60.0 / 3.6
        else:
            # Default case: Germany (big country, source of many automobile constuctors, use international units.)
            # ref. http://wiki.openstreetmap.org/wiki/OSM_tags_for_routing/Maxspeed#Germany
            if 'motorway' in road.type.lower():
                return 130.0 / 3.6  # technically, this is not a maxspeed, but a recommended speed.
            elif 'living_street' in road.type.lower():
                return 10.0 / 3.6
            elif 'bicycle' in road.type.lower():
                return 30.0 / 3.6
            else:
                if road.is_inside_a_city():
                    return 50.0 / 3.6
                else:
                    return 100.0 / 3.6

    milesFactor = 1.60934
    # ref: http://wiki.openstreetmap.org/wiki/Speed_limits#Country_code.2Fcategory_conversion_table
    speedTags = {
        'AT:urban': 50,
        'AT:rural': 100,
        'AT:trunk': 100,
        'AT:motorway': 130,
        'CH:urban': 50,
        'CH:rural': 80,
        'CH:trunk': 100,
        'CH:motorway': 120,
        'CZ:urban': 50,
        'CZ:rural': 90,
        'CZ:trunk': (130, 80),  # 130 in a city, 80 outside a city.
        'CZ:motorway': (130, 80),  # 130 in a city, 80 outside a city.
        'DK:urban': 50,
        'DK:rural': 80,
        'DK:motorway': 130,
        'DE:living_street': 7,
        'DE:urban': 50,
        'DE:rural': 100,
        # 'DE:motorway': ?,  # this case is not documented well in OSM
        'FI:urban': 50,
        'FI:rural': 80,
        'FI:trunk': 100,
        'FI:motorway': 120,
        'FR:urban': 50,
        'FR:rural': 90,
        'FR:trunk': 110,
        'FR:motorway': 130,
        'GR:urban': 50,
        'GR:rural': 90,
        'GR:trunk': 110,
        'GR:motorway': 130,
        'HU:urban': 50,
        'HU:rural': 90,
        'HU:trunk': 110,
        'HU:motorway': 130,
        'IT:urban': 50,
        'IT:rural': 90,
        'IT:trunk': 110,
        'IT:motorway': 130,
        'JP:national': 60,
        'JP:motorway': 100,
        'PL:living_street': 20,
        'PL:urban': 50,
        'PL:rural': 90,
        'PL:motorway': 140,
        'RO:urban': 50,
        'RO:rural': 90,
        'RO:trunk': 100,
        'RO:motorway': 130,
        'RU:living_street': 20,
        'RU:rural': 90,
        'RU:urban': 60,
        'RU:motorway': 110,
        'SK:urban': 50,
        'SK:rural': 90,
        'SK:trunk': (130, 90),  # 130 in a city, 90 outside a city.
        'SK:motorway': (130, 90),  # 130 in a city, 90 outside a city.
        'SI:urban': 50,
        'SI:rural': 90,
        'SI:trunk': 110,
        'SI:motorway': 130,
        'ES:urban': 50,
        'ES:rural': 90,
        'ES:trunk': 100,
        'ES:motorway': 120,
        'SE:urban': 50,
        'SE:rural': 70,
        'SE:trunk': 90,
        'SE:motorway': 110,
        'GB:nsl_single': 60 * milesFactor,
        'GB:nsl_dual': 70 * milesFactor,
        'GB:motorway': 70 * milesFactor,
        'UA:urban': 60,
        'UA:rural': 90,
        'UA:trunk': 110,
        'UA:motorway': 130,
        'UZ:living_street': 30,
        'UZ:urban': 70,
        'UZ:rural': 100,
        'UZ:motorway': 110
    }
