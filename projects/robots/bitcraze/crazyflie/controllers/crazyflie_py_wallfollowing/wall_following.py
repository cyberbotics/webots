#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
#     ||          ____  _ __
#  +------+      / __ )(_) /_______________ _____  ___
#  | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#   ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
#
#  Copyright (C) 2023 Bitcraze AB
#
#  Crazyflie Python Library
#
#  This program is free software; you can redistribute it and/or
#  modify it under the terms of the GNU General Public License
#  as published by the Free Software Foundation; either version 2
#  of the License, or (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <https://www.gnu.org/licenses/>.
"""
Helper class for the wall following demo

This is a python port of c-based app layer example from the Crazyflie-firmware
found here https://github.com/bitcraze/crazyflie-firmware/tree/master/examples/
demos/app_wall_following_demo

"""
import math
from enum import Enum


class WallFollowing():
    class StateWF(Enum):
        FORWARD = 1
        HOVER = 2
        TURN_TO_FIND_WALL = 3
        TURN_TO_ALIGN_TO_WALL = 4
        FORWARD_ALONG_WALL = 5
        ROTATE_AROUND_WALL = 6
        ROTATE_IN_CORNER = 7
        FIND_CORNER = 8

    def __init__(self, ref_distance_from_wall=0.0,
                 max_forward_speed=0.2,
                 max_turn_rate=0.5,
                 direction=1.0,
                 first_run=False,
                 prev_heading=0.0,
                 wall_angle=0.0,
                 around_corner_back_track=False,
                 state_start_time=0.0,
                 ranger_value_buffer=0.2,
                 angle_value_buffer=0.1,
                 ranger_threshold_lost=0.3,
                 in_corner_angle=0.8,
                 wait_for_measurement_seconds=1.0,
                 init_state=StateWF.FORWARD):

        self.ref_distance_from_wall = ref_distance_from_wall
        self.max_forward_speed = max_forward_speed
        self.max_turn_rate = max_turn_rate
        self.direction = direction
        self.first_run = first_run
        self.prev_heading = prev_heading
        self.wall_angle = wall_angle
        self.around_corner_back_track = around_corner_back_track
        self.state_start_time = state_start_time
        self.ranger_value_buffer = ranger_value_buffer
        self.angle_value_buffer = angle_value_buffer
        self.ranger_threshold_lost = ranger_threshold_lost
        self.in_corner_angle = in_corner_angle
        self.wait_for_measurement_seconds = wait_for_measurement_seconds

        self.first_run = True
        self.state_wf = init_state
        self.time_now = 0.0

    def logic_is_close_to(self, real_value, checked_value, margin):
        if real_value > checked_value - margin and real_value < checked_value + margin:
            return True
        else:
            return False

    def wrap_to_pi(self, number):
        if number > math.pi:
            return number - 2 * math.pi
        elif number < -math.pi:
            return number + 2 * math.pi
        else:
            return number

    def command_turn(self, ref_rate):
        cmd_vel_x = 0.0
        cmd_ang_w = self.direction * ref_rate
        return cmd_vel_x, cmd_ang_w

    def command_align_corner(self, ref_rate, range, wanted_distance_from_corner):
        if range > wanted_distance_from_corner + 0.3:
            cmd_ang_w = self.direction * ref_rate
            cmd_vel_y = 0
        else:
            if range > wanted_distance_from_corner:
                cmd_vel_y = self.direction * (-1.0 * self.max_forward_speed / 3.0)
            else:
                cmd_vel_y = self.direction * (self.max_forward_speed / 3.0)
            cmd_ang_w = 0
        return cmd_vel_y, cmd_ang_w

    def command_hover(self):
        cmd_vel_x = 0.0
        cmd_vel_y = 0.0
        cmd_ang_w = 0.0
        return cmd_vel_x, cmd_vel_y, cmd_ang_w

    def command_forward_along_wall(self, range):
        cmd_vel_x = self.max_forward_speed
        cmd_vel_y = 0
        check_distance_wall = self.logic_is_close_to(self.ref_distance_from_wall, range, 0.1)
        if not check_distance_wall:
            if range > self.ref_distance_from_wall:
                cmd_vel_y = self.direction * (-1.0 * self.max_forward_speed / 2.0)
            else:
                cmd_vel_y = self.direction * (self.max_forward_speed / 2.0)
        return cmd_vel_x, cmd_vel_y

    def command_turn_around_corner_and_adjust(self, radius, range):
        cmd_vel_x = self.max_forward_speed
        cmd_ang_w = self.direction * (-1 * cmd_vel_x / radius)
        cmd_vel_y = 0.0
        check_distance_wall = self.logic_is_close_to(self.ref_distance_from_wall, range, 0.1)
        if not check_distance_wall:
            if range > self.ref_distance_from_wall:
                cmd_vel_y = self.direction * (-1.0 * self.max_forward_speed / 3.0)
            else:
                cmd_vel_y = self.direction * (self.max_forward_speed / 3.0)
        return cmd_vel_x, cmd_vel_y, cmd_ang_w

    def command_turn_and_adjust(self, rate, range):
        cmd_ang_w = self.direction * rate
        cmd_vel_y = 0.0
        return cmd_vel_y, cmd_ang_w

    def transition(self, new_state):
        self.state_start_time = self.time_now
        return new_state

    def adjust_distance_wall(self, distance_wall_new):
        self.ref_distance_from_wall = distance_wall_new

    def wall_follower(self, front_range, side_range, current_heading,
                      direction_turn, time_outer_loop):

        self.direction = direction_turn
        self.time_now = time_outer_loop

        if self.first_run:
            self.prev_heading = current_heading
            self.around_corner_back_track = False
            self.first_run = False

        # Handle state transitions
        match self.state_wf:
            case self.StateWF.FORWARD:
                if front_range < self.ref_distance_from_wall + self.ranger_value_buffer:
                    self.state_wf = self.transition(self.StateWF.TURN_TO_FIND_WALL)
            case self.StateWF.HOVER:
                print('hover')
            case self.StateWF.TURN_TO_FIND_WALL:
                side_range_check = side_range < (self.ref_distance_from_wall /
                                                 math.cos(0.78) + self.ranger_value_buffer)
                front_range_check = front_range < (self.ref_distance_from_wall /
                                                   math.cos(0.78) + self.ranger_value_buffer)
                if side_range_check and front_range_check:
                    self.prev_heading = current_heading
                    self.wall_angle = self.direction * \
                        (1.57 - math.atan(front_range / side_range) + self.angle_value_buffer)
                    self.state_wf = self.transition(self.StateWF.TURN_TO_ALIGN_TO_WALL)
                # If went too far in heading
                if side_range < self.ref_distance_from_wall + self.ranger_value_buffer and \
                        front_range > self.ref_distance_from_wall + self.ranger_threshold_lost:
                    self.around_corner_back_track = False
                    self.prev_heading = current_heading
                    self.state_wf = self.transition(self.StateWF.FIND_CORNER)
            case self.StateWF.TURN_TO_ALIGN_TO_WALL:
                align_wall_check = self.logic_is_close_to(
                    self.wrap_to_pi(current_heading - self.prev_heading), self.wall_angle, self.angle_value_buffer)
                if align_wall_check:
                    self.state_wf = self.transition(self.StateWF.FORWARD_ALONG_WALL)
            case self.StateWF.FORWARD_ALONG_WALL:
                # If side range is out of reach,
                #    end of the wall is reached
                if side_range > self.ref_distance_from_wall + self.ranger_threshold_lost:
                    self.state_wf = self.transition(self.StateWF.FIND_CORNER)
                # If front range is small
                #    then corner is reached
                if front_range < self.ref_distance_from_wall + self.ranger_value_buffer:
                    self.prev_heading = current_heading
                    self.state_wf = self.transition(self.StateWF.ROTATE_IN_CORNER)
            case self.StateWF.ROTATE_AROUND_WALL:
                if front_range < self.ref_distance_from_wall + self.ranger_value_buffer:
                    self.state_wf = self.transition(self.StateWF.TURN_TO_FIND_WALL)

            case self.StateWF.ROTATE_IN_CORNER:
                check_heading_corner = self.logic_is_close_to(
                    math.fabs(self.wrap_to_pi(current_heading-self.prev_heading)),
                    self.in_corner_angle, self.angle_value_buffer)
                if check_heading_corner:
                    self.state_wf = self.transition(self.StateWF.TURN_TO_FIND_WALL)
            case self.StateWF.FIND_CORNER:
                if side_range <= self.ref_distance_from_wall:
                    self.state_wf = self.transition(self.StateWF.ROTATE_AROUND_WALL)
            case _:
                self.state_wf = self.transition(self.StateWF.HOVER)

        # Handle state actions
        cmd_vel_x_temp = 0.0
        cmd_vel_y_temp = 0.0
        cmd_ang_w_temp = 0.0

        match self.state_wf:
            case self.StateWF.FORWARD:
                cmd_vel_x_temp = self.max_forward_speed
                cmd_vel_y_temp = 0.0
                cmd_ang_w_temp = 0.0
            case self.StateWF.HOVER:
                cmd_vel_x_temp, cmd_vel_y_temp, cmd_ang_w_temp = self.command_hover()
            case self.StateWF.TURN_TO_FIND_WALL:
                cmd_vel_x_temp, cmd_ang_w_temp = self.command_turn(self.max_turn_rate)
                cmd_vel_y_temp = 0.0
            case self.StateWF.TURN_TO_ALIGN_TO_WALL:
                if self.time_now - self.state_start_time < self.wait_for_measurement_seconds:
                    cmd_vel_x_temp, cmd_vel_y_temp, cmd_ang_w_temp = self.command_hover()
                else:
                    cmd_vel_x_temp, cmd_ang_w_temp = self.command_turn(self.max_turn_rate)
                    cmd_vel_y_temp = 0.0
            case self.StateWF.FORWARD_ALONG_WALL:
                cmd_vel_x_temp, cmd_vel_y_temp = self.command_forward_along_wall(side_range)
                cmd_ang_w_temp = 0.0
            case self.StateWF.ROTATE_AROUND_WALL:
                # If first time around corner
                #   first try to find the wall again

                # if side range is larger than preffered distance from wall
                if side_range > self.ref_distance_from_wall + self.ranger_threshold_lost:
                    # check if scanning already occured
                    if self.wrap_to_pi(math.fabs(current_heading - self.prev_heading)) > \
                            self.in_corner_angle:
                        self.around_corner_back_track = True
                    # turn and adjust distance to corner from that point
                    if self.around_corner_back_track:
                        # rotate back if it already went into one direction
                        cmd_vel_y_temp, cmd_ang_w_temp = self.command_turn_and_adjust(
                            -1 * self.max_turn_rate, side_range)
                        cmd_vel_x_temp = 0.0
                    else:
                        cmd_vel_y_temp, cmd_ang_w_temp = self.command_turn_and_adjust(self.max_turn_rate, side_range)
                        cmd_vel_x_temp = 0.0
                else:
                    # continue to turn around corner
                    self.prev_heading = current_heading
                    self.around_corner_back_track = False
                    cmd_vel_x_temp, cmd_vel_y_temp, cmd_ang_w_temp = \
                        self.command_turn_around_corner_and_adjust(
                            self.ref_distance_from_wall, side_range)
            case self.StateWF.ROTATE_IN_CORNER:
                cmd_vel_x_temp, cmd_ang_w_temp = self.command_turn(self.max_turn_rate)
                cmd_vel_y_temp = 0.0
            case self.StateWF.FIND_CORNER:
                cmd_vel_y_temp, cmd_ang_w_temp = self.command_align_corner(
                    -1 * self.max_turn_rate, side_range, self.ref_distance_from_wall)
                cmd_vel_x_temp = 0.0
            case _:
                # state does not exist, so hover!
                cmd_vel_x_temp, cmd_vel_y_temp, cmd_ang_w_temp = self.command_hover()

        cmd_vel_x = cmd_vel_x_temp
        cmd_vel_y = cmd_vel_y_temp
        cmd_ang_w = cmd_ang_w_temp

        return cmd_vel_x, cmd_vel_y, cmd_ang_w, self.state_wf
