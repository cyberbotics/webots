# -*- coding: utf-8 -*-
#
#  ...........       ____  _ __
#  |  ,-^-,  |      / __ )(_) /_______________ _____  ___
#  | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  | / ,..´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#     +.......   /_____/_/\__/\___/_/   \__,_/ /___/\___/
#
#  GNU general public license v3.0
#
#  Copyright (C) 2023 Bitcraze AB
#

"""
file: wall_following.py

Class for the wall following demo

This is a python port of c-based app layer example from the Crazyflie-firmware
found here https://github.com/bitcraze/crazyflie-firmware/tree/master/examples/
demos/app_wall_following_demo

Author:   Kimberly McGuire (Bitcraze AB)
"""


import math
from enum import Enum


class WallFollowing():
    class StateWallFollowing(Enum):
        FORWARD = 1
        HOVER = 2
        TURN_TO_FIND_WALL = 3
        TURN_TO_ALIGN_TO_WALL = 4
        FORWARD_ALONG_WALL = 5
        ROTATE_AROUND_WALL = 6
        ROTATE_IN_CORNER = 7
        FIND_CORNER = 8

    class WallFollowingDirection(Enum):
        LEFT = 1
        RIGHT = -1

    def __init__(self, reference_distance_from_wall=0.0,
                 max_forward_speed=0.2,
                 max_turn_rate=0.5,
                 wall_following_direction=WallFollowingDirection.LEFT,
                 first_run=False,
                 prev_heading=0.0,
                 wall_angle=0.0,
                 around_corner_back_track=False,
                 state_start_time=0.0,
                 ranger_value_buffer=0.2,
                 angle_value_buffer=0.1,
                 range_lost_threshold=0.3,
                 in_corner_angle=0.8,
                 wait_for_measurement_seconds=1.0,
                 init_state=StateWallFollowing.FORWARD):
        """
        __init__ function for the WallFollowing class

        reference_distance_from_wall is the distance from the wall that the Crazyflie
            should try to keep
        max_forward_speed is the maximum speed the Crazyflie should fly forward
        max_turn_rate is the maximum turn rate the Crazyflie should turn with
        wall_following_direction is the direction the Crazyflie should follow the wall
            (WallFollowingDirection Enum)
        first_run is a boolean that is True if the Crazyflie is in the first run of the
            wall following demo
        prev_heading is the heading of the Crazyflie in the previous state (in rad)
        wall_angle is the angle of the wall in the previous state (in rad)
        around_corner_back_track is a boolean that is True if the Crazyflie is in the
            around corner state and should back track
        state_start_time is the time when the Crazyflie entered the current state (in s)
        ranger_value_buffer is the buffer value for the ranger measurements (in m)
        angle_value_buffer is the buffer value for the angle measurements (in rad)
        range_lost_threshold is the threshold for when the Crazyflie should stop
            following the wall (in m)
        in_corner_angle is the angle the Crazyflie should turn when it is in the corner (in rad)
        wait_for_measurement_seconds is the time the Crazyflie should wait for a
            measurement before it starts the wall following demo (in s)
        init_state is the initial state of the Crazyflie (StateWallFollowing Enum)
        self.state is a shared state variable that is used to keep track of the current
            state of the Crazyflie's wall following
        self.time_now is a shared state variable that is used to keep track of the current (in s)
        """

        self.reference_distance_from_wall = reference_distance_from_wall
        self.max_forward_speed = max_forward_speed
        self.max_turn_rate = max_turn_rate
        self.wall_following_direction_value = float(wall_following_direction.value)
        self.first_run = first_run
        self.prev_heading = prev_heading
        self.wall_angle = wall_angle
        self.around_corner_back_track = around_corner_back_track
        self.state_start_time = state_start_time
        self.ranger_value_buffer = ranger_value_buffer
        self.angle_value_buffer = angle_value_buffer
        self.range_threshold_lost = range_lost_threshold
        self.in_corner_angle = in_corner_angle
        self.wait_for_measurement_seconds = wait_for_measurement_seconds

        self.first_run = True
        self.state = init_state
        self.time_now = 0.0
        self.speed_redux_corner = 3.0
        self.speed_redux_straight = 2.0

    # Helper function
    def value_is_close_to(self, real_value, checked_value, margin):
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

    # Command functions
    def command_turn(self, reference_rate):
        """
        Command the Crazyflie to turn around its yaw axis

        reference_rate and rate_yaw is defined in rad/s
        velocity_x is defined in m/s
        """
        velocity_x = 0.0
        rate_yaw = self.wall_following_direction_value * reference_rate
        return velocity_x, rate_yaw

    def command_align_corner(self, reference_rate, side_range, wanted_distance_from_corner):
        """
        Command the Crazyflie to align itself to the outer corner
            and make sure it's at a certain distance from it

        side_range and wanted_distance_from_corner is defined in m
        reference_rate and rate_yaw is defined in rad/s
        velocity_x is defined in m/s
        """
        if side_range > wanted_distance_from_corner + self.range_threshold_lost:
            rate_yaw = self.wall_following_direction_value * reference_rate
            velocity_y = 0.0
        else:
            if side_range > wanted_distance_from_corner:
                velocity_y = self.wall_following_direction_value * \
                    (-1.0 * self.max_forward_speed / self.speed_redux_corner)
            else:
                velocity_y = self.wall_following_direction_value * (self.max_forward_speed / self.speed_redux_corner)
            rate_yaw = 0.0
        return velocity_y, rate_yaw

    def command_hover(self):
        """
        Command the Crazyflie to hover in place
        """
        velocity_x = 0.0
        velocity_y = 0.0
        rate_yaw = 0.0
        return velocity_x, velocity_y, rate_yaw

    def command_forward_along_wall(self, side_range):
        """
        Command the Crazyflie to fly forward along the wall
            while controlling it's distance to it

        side_range is defined in m
        velocity_x and velocity_y is defined in m/s
        """
        velocity_x = self.max_forward_speed
        velocity_y = 0.0
        check_distance_wall = self.value_is_close_to(
            self.reference_distance_from_wall, side_range, self.ranger_value_buffer)
        if not check_distance_wall:
            if side_range > self.reference_distance_from_wall:
                velocity_y = self.wall_following_direction_value * \
                    (-1.0 * self.max_forward_speed / self.speed_redux_straight)
            else:
                velocity_y = self.wall_following_direction_value * (self.max_forward_speed / self.speed_redux_straight)
        return velocity_x, velocity_y

    def command_turn_around_corner_and_adjust(self, radius, side_range):
        """
        Command the Crazyflie to turn around the corner
            and adjust it's distance to the corner

        radius is defined in m
        side_range is defined in m
        velocity_x and velocity_y is defined in m/s
        """
        velocity_x = self.max_forward_speed
        rate_yaw = self.wall_following_direction_value * (-1 * velocity_x / radius)
        velocity_y = 0.0
        check_distance_wall = self.value_is_close_to(
            self.reference_distance_from_wall, side_range, self.ranger_value_buffer)
        if not check_distance_wall:
            if side_range > self.reference_distance_from_wall:
                velocity_y = self.wall_following_direction_value * \
                    (-1.0 * self.max_forward_speed / self.speed_redux_corner)
            else:
                velocity_y = self.wall_following_direction_value * (self.max_forward_speed / self.speed_redux_corner)
        return velocity_x, velocity_y, rate_yaw

    # state machine helper functions
    def state_transition(self, new_state):
        """
        Transition to a new state and reset the state timer

        new_state is defined in the StateWallFollowing enum
        """
        self.state_start_time = self.time_now
        return new_state

    def adjust_reference_distance_wall(self, reference_distance_wall_new):
        """
        Adjust the reference distance to the wall
        """
        self.reference_distance_from_wall = reference_distance_wall_new

    # Wall following State machine
    def wall_follower(self, front_range, side_range, current_heading,
                      wall_following_direction, time_outer_loop):
        """
        wall_follower is the main function of the wall following state machine.
        It takes the current range measurements of the front range and side range
        sensors, the current heading of the Crazyflie, the wall following direction
        and the current time of the outer loop (the real time or the simulation time)
        as input, and handles the state transitions and commands the Crazyflie to
        to do the wall following.

        front_range and side_range is defined in m
        current_heading is defined in rad
        wall_following_direction is defined as WallFollowingDirection enum
        time_outer_loop is defined in seconds (double)
        command_velocity_x, command_velocity_ y is defined in m/s
        command_rate_yaw is defined in rad/s
        self.state is defined as StateWallFollowing enum
        """

        self.wall_following_direction_value = float(wall_following_direction.value)
        self.time_now = time_outer_loop

        if self.first_run:
            self.prev_heading = current_heading
            self.around_corner_back_track = False
            self.first_run = False

        # -------------- Handle state transitions ---------------- #
        if self.state == self.StateWallFollowing.FORWARD:
            if front_range < self.reference_distance_from_wall + self.ranger_value_buffer:
                self.state = self.state_transition(self.StateWallFollowing.TURN_TO_FIND_WALL)
        elif self.state == self.StateWallFollowing.HOVER:
            print('hover')
        elif self.state == self.StateWallFollowing.TURN_TO_FIND_WALL:
            # Turn until 45 degrees from wall such that the front and side range sensors
            #   can detect the wall
            side_range_check = side_range < (self.reference_distance_from_wall /
                                             math.cos(math.pi/4) + self.ranger_value_buffer)
            front_range_check = front_range < (self.reference_distance_from_wall /
                                               math.cos(math.pi/4) + self.ranger_value_buffer)
            if side_range_check and front_range_check:
                self.prev_heading = current_heading
                # Calculate the angle to the wall
                self.wall_angle = self.wall_following_direction_value * \
                    (math.pi/2 - math.atan(front_range / side_range) + self.angle_value_buffer)
                self.state = self.state_transition(self.StateWallFollowing.TURN_TO_ALIGN_TO_WALL)
            # If went too far in heading and lost the wall, go to find corner.
            if side_range < self.reference_distance_from_wall + self.ranger_value_buffer and \
                    front_range > self.reference_distance_from_wall + self.range_threshold_lost:
                self.around_corner_back_track = False
                self.prev_heading = current_heading
                self.state = self.state_transition(self.StateWallFollowing.FIND_CORNER)
        elif self.state == self.StateWallFollowing.TURN_TO_ALIGN_TO_WALL:
            align_wall_check = self.value_is_close_to(
                self.wrap_to_pi(current_heading - self.prev_heading), self.wall_angle, self.angle_value_buffer)
            if align_wall_check:
                self.state = self.state_transition(self.StateWallFollowing.FORWARD_ALONG_WALL)
        elif self.state == self.StateWallFollowing.FORWARD_ALONG_WALL:
            # If side range is out of reach,
            #    end of the wall is reached
            if side_range > self.reference_distance_from_wall + self.range_threshold_lost:
                self.state = self.state_transition(self.StateWallFollowing.FIND_CORNER)
            # If front range is small
            #    then corner is reached
            if front_range < self.reference_distance_from_wall + self.ranger_value_buffer:
                self.prev_heading = current_heading
                self.state = self.state_transition(self.StateWallFollowing.ROTATE_IN_CORNER)
        elif self.state == self.StateWallFollowing.ROTATE_AROUND_WALL:
            if front_range < self.reference_distance_from_wall + self.ranger_value_buffer:
                self.state = self.state_transition(self.StateWallFollowing.TURN_TO_FIND_WALL)
        elif self.state == self.StateWallFollowing.ROTATE_IN_CORNER:
            check_heading_corner = self.value_is_close_to(
                math.fabs(self.wrap_to_pi(current_heading-self.prev_heading)),
                self.in_corner_angle, self.angle_value_buffer)
            if check_heading_corner:
                self.state = self.state_transition(self.StateWallFollowing.TURN_TO_FIND_WALL)
        elif self.state == self.StateWallFollowing.FIND_CORNER:
            if side_range <= self.reference_distance_from_wall:
                self.state = self.state_transition(self.StateWallFollowing.ROTATE_AROUND_WALL)
        else:
            self.state = self.state_transition(self.StateWallFollowing.HOVER)

        # -------------- Handle state actions ---------------- #
        command_velocity_x_temp = 0.0
        command_velocity_y_temp = 0.0
        command_angle_rate_temp = 0.0

        if self.state == self.StateWallFollowing.FORWARD:
            command_velocity_x_temp = self.max_forward_speed
            command_velocity_y_temp = 0.0
            command_angle_rate_temp = 0.0
        elif self.state == self.StateWallFollowing.HOVER:
            command_velocity_x_temp, command_velocity_y_temp, command_angle_rate_temp = self.command_hover()
        elif self.state == self.StateWallFollowing.TURN_TO_FIND_WALL:
            command_velocity_x_temp, command_angle_rate_temp = self.command_turn(self.max_turn_rate)
            command_velocity_y_temp = 0.0
        elif self.state == self.StateWallFollowing.TURN_TO_ALIGN_TO_WALL:
            if self.time_now - self.state_start_time < self.wait_for_measurement_seconds:
                command_velocity_x_temp, command_velocity_y_temp, command_angle_rate_temp = self.command_hover()
            else:
                command_velocity_x_temp, command_angle_rate_temp = self.command_turn(self.max_turn_rate)
                command_velocity_y_temp = 0.0
        elif self.state == self.StateWallFollowing.FORWARD_ALONG_WALL:
            command_velocity_x_temp, command_velocity_y_temp = self.command_forward_along_wall(side_range)
            command_angle_rate_temp = 0.0
        elif self.state == self.StateWallFollowing.ROTATE_AROUND_WALL:
            # If first time around corner
            #   first try to find the wall again
            # if side range is larger than preffered distance from wall
            if side_range > self.reference_distance_from_wall + self.range_threshold_lost:
                # check if scanning already occured
                if self.wrap_to_pi(math.fabs(current_heading - self.prev_heading)) > \
                        self.in_corner_angle:
                    self.around_corner_back_track = True
                # turn and adjust distance to corner from that point
                if self.around_corner_back_track:
                    # rotate back if it already went into one direction
                    command_velocity_y_temp, command_angle_rate_temp = self.command_turn(
                        -1 * self.max_turn_rate)
                    command_velocity_x_temp = 0.0
                else:
                    command_velocity_y_temp, command_angle_rate_temp = self.command_turn(
                        self.max_turn_rate)
                    command_velocity_x_temp = 0.0
            else:
                # continue to turn around corner
                self.prev_heading = current_heading
                self.around_corner_back_track = False
                command_velocity_x_temp, command_velocity_y_temp, command_angle_rate_temp = \
                    self.command_turn_around_corner_and_adjust(
                        self.reference_distance_from_wall, side_range)
        elif self.state == self.StateWallFollowing.ROTATE_IN_CORNER:
            command_velocity_x_temp, command_angle_rate_temp = self.command_turn(self.max_turn_rate)
            command_velocity_y_temp = 0.0
        elif self.state == self.StateWallFollowing.FIND_CORNER:
            command_velocity_y_temp, command_angle_rate_temp = self.command_align_corner(
                -1 * self.max_turn_rate, side_range, self.reference_distance_from_wall)
            command_velocity_x_temp = 0.0
        else:
            # state does not exist, so hover!
            command_velocity_x_temp, command_velocity_y_temp, command_angle_rate_temp = self.command_hover()

        command_velocity_x = command_velocity_x_temp
        command_velocity_y = command_velocity_y_temp
        command_yaw_rate = command_angle_rate_temp

        return command_velocity_x, command_velocity_y, command_yaw_rate, self.state
