# Copyright 2017 Open Source Robotics Foundation, Inc.
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

from catkin_pkg.condition import evaluate_condition


class GroupMembership:
    __slots__ = [
        'name',
        'condition',
        'evaluated_condition',
    ]

    def __init__(self, name, condition=None):
        self.name = name
        self.condition = condition
        self.evaluated_condition = None

    def __eq__(self, other):
        if not isinstance(other, GroupMembership):
            return False
        return all(getattr(self, attr) == getattr(other, attr)
                   for attr in self.__slots__)

    def __str__(self):
        return self.name

    def evaluate_condition(self, context):
        """
        Evaluate the condition.

        The result is also stored in the member variable `evaluated_condition`.

        :param context: A dictionary with key value pairs to replace variables
          starting with $ in the condition.

        :returns: True if the condition evaluates to True, else False
        :raises: :exc:`ValueError` if the condition fails to parse
        """
        self.evaluated_condition = evaluate_condition(self.condition, context)
        return self.evaluated_condition
